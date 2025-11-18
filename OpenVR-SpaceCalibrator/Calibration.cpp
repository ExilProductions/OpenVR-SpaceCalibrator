#include "Calibration.h"
#include "Configuration.h"
#include "IPCClient.h"
#include "CalibrationCalc.h"

#include <string>
#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include <GLFW/glfw3.h>


inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

// Convert driver pose from driver space to world space
// This is CRITICAL for proper calibration!
Pose ConvertPose(const vr::DriverPose_t &driverPose) {
	Eigen::Quaterniond driverToWorldQ(
		driverPose.qWorldFromDriverRotation.w,
		driverPose.qWorldFromDriverRotation.x,
		driverPose.qWorldFromDriverRotation.y,
		driverPose.qWorldFromDriverRotation.z
	);
	Eigen::Vector3d driverToWorldV(
		driverPose.vecWorldFromDriverTranslation[0],
		driverPose.vecWorldFromDriverTranslation[1],
		driverPose.vecWorldFromDriverTranslation[2]
	);

	// Transform device rotation from driver space to world space
	Eigen::Quaterniond driverRot = driverToWorldQ * Eigen::Quaterniond(
		driverPose.qRotation.w,
		driverPose.qRotation.x,
		driverPose.qRotation.y,
		driverPose.qRotation.z
	);

	// Transform device position from driver space to world space
	Eigen::Vector3d driverPos = driverToWorldV + driverToWorldQ * Eigen::Vector3d(
		driverPose.vecPosition[0],
		driverPose.vecPosition[1],
		driverPose.vecPosition[2]
	);

	Eigen::AffineCompact3d xform = Eigen::Translation3d(driverPos) * driverRot;

	return Pose(xform);
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
	vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
	vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}

inline Eigen::Matrix3d quaternionRotateMatrix(const vr::HmdQuaternion_t& quat) {
	return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z).toRotationMatrix();
}


static IPCClient Driver;
CalibrationContext CalCtx;
static CalibrationCalc calibration;
static protocol::DriverPoseShmem shmem;

namespace {
	// Simplified AssignTargets for Linux - validates current device IDs
	// Full VRState-based device discovery not ported yet
	bool AssignTargets() {
		// For now, just validate that the IDs are in valid range
		// The UI already sets these IDs when user selects devices
		if (CalCtx.referenceID < 0 || CalCtx.referenceID >= vr::k_unMaxTrackedDeviceCount) {
			return false;
		}
		if (CalCtx.targetID < 0 || CalCtx.targetID >= vr::k_unMaxTrackedDeviceCount) {
			return false;
		}

		// Note: In Windows version, this also populates controllerIDs for trigger press support
		// That functionality can be added later if needed

		return true;
	}
}

void InitCalibrator()
{
	Driver.Connect();

	// Open shared memory for reading driver poses
	if (!shmem.Open(OPENVR_SPACECALIBRATOR_SHMEM_NAME)) {
		std::cout << "Warning: Could not open pose shared memory. Will fall back to VR API." << std::endl;
	} else {
		std::cout << "Successfully opened pose shared memory" << std::endl;
	}

	// Initialize driver pose array
	memset(CalCtx.driverPoses, 0, sizeof(CalCtx.driverPoses));
}

struct DSample
{
	bool valid;
	Eigen::Vector3d ref, target;
};

bool StartsWith(const std::string &str, const std::string &prefix)
{
	if (str.length() < prefix.length())
		return false;

	return str.compare(0, prefix.length(), prefix) == 0;
}

bool EndsWith(const std::string &str, const std::string &suffix)
{
	if (str.length() < suffix.length())
		return false;

	return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

Eigen::Vector3d AxisFromRotationMatrix3(Eigen::Matrix3d rot)
{
	return Eigen::Vector3d(rot(2,1) - rot(1,2), rot(0,2) - rot(2,0), rot(1,0) - rot(0,1));
}

double AngleFromRotationMatrix3(Eigen::Matrix3d rot)
{
	return acos((rot(0,0) + rot(1,1) + rot(2,2) - 1.0) / 2.0);
}

DSample DeltaRotationSamples(Sample s1, Sample s2)
{
	// Difference in rotation between samples.
	auto dref = s1.ref.rot * s2.ref.rot.transpose();
	auto dtarget = s1.target.rot * s2.target.rot.transpose();

	// When stuck together, the two tracked objects rotate as a pair,
	// therefore their axes of rotation must be equal between any given pair of samples.
	DSample ds;
	ds.ref = AxisFromRotationMatrix3(dref);
	ds.target = AxisFromRotationMatrix3(dtarget);

	// Reject samples that were too close to each other.
	auto refA = AngleFromRotationMatrix3(dref);
	auto targetA = AngleFromRotationMatrix3(dtarget);
	ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

	ds.ref.normalize();
	ds.target.normalize();
	return ds;
}

Eigen::Vector3d CalibrateRotation(const std::vector<Sample> &samples)
{
	std::vector<DSample> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			auto delta = DeltaRotationSamples(samples[i], samples[j]);
			if (delta.valid)
				deltas.push_back(delta);
		}
	}
	char buf[256];
	snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", samples.size(), deltas.size());
	CalCtx.Log(buf);

	// Kabsch algorithm

	Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);
	Eigen::Vector3d refCentroid(0,0,0), targetCentroid(0,0,0);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) = deltas[i].ref;
		refCentroid += deltas[i].ref;

		targetPoints.row(i) = deltas[i].target;
		targetCentroid += deltas[i].target;
	}

	refCentroid /= (double) deltas.size();
	targetCentroid /= (double) deltas.size();

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) -= refCentroid;
		targetPoints.row(i) -= targetCentroid;
	}

	auto crossCV = refPoints.transpose() * targetPoints;

	Eigen::BDCSVD<Eigen::MatrixXd> bdcsvd;
	auto svd = bdcsvd.compute(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
	if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0)
	{
		i(2,2) = -1;
	}

	Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
	rot.transposeInPlace();

	Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;

	snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
	CalCtx.Log(buf);
	return euler;
}

Eigen::Vector3d CalibrateTranslation(const std::vector<Sample> &samples)
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			auto QAi = samples[i].ref.rot.transpose();
			auto QAj = samples[j].ref.rot.transpose();
			auto dQA = QAj - QAi;
			auto CA = QAj * (samples[j].ref.trans - samples[j].target.trans) - QAi * (samples[i].ref.trans - samples[i].target.trans);
			deltas.push_back(std::make_pair(CA, dQA));

			auto QBi = samples[i].target.rot.transpose();
			auto QBj = samples[j].target.rot.transpose();
			auto dQB = QBj - QBi;
			auto CB = QBj * (samples[j].ref.trans - samples[j].target.trans) - QBi * (samples[i].ref.trans - samples[i].target.trans);
			deltas.push_back(std::make_pair(CB, dQB));
		}
	}

	Eigen::VectorXd constants(deltas.size() * 3);
	Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		for (int axis = 0; axis < 3; axis++)
		{
			constants(i * 3 + axis) = deltas[i].first(axis);
			coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
		}
	}

	Eigen::Vector3d trans = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
	auto transcm = trans * 100.0;

	char buf[256];
	snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
	CalCtx.Log(buf);
	return transcm;
}

Sample CollectSample(const CalibrationContext &ctx)
{
	vr::DriverPose_t reference, target;
	reference.poseIsValid = false;
	target.poseIsValid = false;

	reference = ctx.driverPoses[ctx.referenceID];
	target = ctx.driverPoses[ctx.targetID];

	bool ok = true;
	if (!reference.poseIsValid)
	{
		CalCtx.Log("Reference device is not tracking\n"); ok = false;
	}
	if (!target.poseIsValid)
	{
		CalCtx.Log("Target device is not tracking\n"); ok = false;
	}
	if (!ok)
	{
		if (ctx.state != CalibrationState::Continuous) {
			CalCtx.Log("Aborting calibration!\n");
			CalCtx.state = CalibrationState::None;
		}
		return Sample();
	}

	// Apply tracker offsets for continuous calibration
	if (ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby) {
		reference.vecPosition[0] += ctx.continuousCalibrationOffset.x();
		reference.vecPosition[1] += ctx.continuousCalibrationOffset.y();
		reference.vecPosition[2] += ctx.continuousCalibrationOffset.z();
	}

	// Convert driver poses to world space poses - CRITICAL!
	return Sample(
		ConvertPose(reference),
		ConvertPose(target),
		glfwGetTime()
	);
}

vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg)
{
	auto euler = eulerdeg * EIGEN_PI / 180.0;

	Eigen::Quaterniond rotQuat =
		Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

	vr::HmdQuaternion_t vrRotQuat;
	vrRotQuat.x = rotQuat.coeffs()[0];
	vrRotQuat.y = rotQuat.coeffs()[1];
	vrRotQuat.z = rotQuat.coeffs()[2];
	vrRotQuat.w = rotQuat.coeffs()[3];
	return vrRotQuat;
}

vr::HmdVector3d_t VRTranslationVec(Eigen::Vector3d transcm)
{
	auto trans = transcm * 0.01;
	vr::HmdVector3d_t vrTrans;
	vrTrans.v[0] = trans[0];
	vrTrans.v[1] = trans[1];
	vrTrans.v[2] = trans[2];
	return vrTrans;
}

void ResetAndDisableOffsets(uint32_t id)
{
	vr::HmdVector3d_t zeroV;
	zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;

	vr::HmdQuaternion_t zeroQ;
	zeroQ.x = 0; zeroQ.y = 0; zeroQ.z = 0; zeroQ.w = 1;

	protocol::Request req(protocol::RequestSetDeviceTransform);
	req.setDeviceTransform = { id, false, zeroV, zeroQ, 1.0 };
	Driver.SendBlocking(req);
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

void ScanAndApplyProfile(CalibrationContext &ctx)
{
	char buffer[vr::k_unMaxPropertyStringSize];
	ctx.enabled = ctx.validProfile;

	// Send alignment speed parameters to driver for smooth interpolation
	protocol::Request setParamsReq(protocol::RequestSetAlignmentSpeedParams);
	setParamsReq.setAlignmentSpeedParams = ctx.alignmentSpeedParams;
	Driver.SendBlocking(setParamsReq);

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		/*if (deviceClass == vr::TrackedDeviceClass_HMD) // for debugging unexpected universe switches
		{
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			auto universeId = vr::VRSystem()->GetUint64TrackedDeviceProperty(id, vr::Prop_CurrentUniverseId_Uint64, &err);
			printf("uid %d err %d\n", universeId, err);
			ResetAndDisableOffsets(id);
			continue;
		}*/

		if (!ctx.enabled)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

		if (err != vr::TrackedProp_Success)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		std::string trackingSystem(buffer);

		if (id == vr::k_unTrackedDeviceIndex_Hmd)
		{
			//auto p = ctx.devicePoses[id].mDeviceToAbsoluteTracking.m;
			//printf("HMD %d: %f %f %f\n", id, p[0][3], p[1][3], p[2][3]);

			if (trackingSystem != ctx.referenceTrackingSystem)
			{
				// Currently using an HMD with a different tracking system than the calibration.
				ctx.enabled = false;
			}

			ResetAndDisableOffsets(id);
			continue;
		}

		if (trackingSystem != ctx.targetTrackingSystem)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		protocol::Request req(protocol::RequestSetDeviceTransform);
		req.setDeviceTransform = {
			id,
			true,
			VRTranslationVec(ctx.calibratedTranslation),
			VRRotationQuat(ctx.calibratedRotation),
			ctx.calibratedScale
		};
		// Enable lerp (smooth interpolation) for continuous calibration
		req.setDeviceTransform.lerp = (CalCtx.state == CalibrationState::Continuous);
		// Quash target pose updates during continuous calibration if enabled
		req.setDeviceTransform.quash = (CalCtx.state == CalibrationState::Continuous && id == CalCtx.targetID && CalCtx.quashTargetInContinuous);
		Driver.SendBlocking(req);
	}

	if (ctx.enabled && ctx.chaperone.valid && ctx.chaperone.autoApply)
	{
		uint32_t quadCount = 0;
		vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

		// Heuristic: when SteamVR resets to a blank-ish chaperone, it uses empty geometry,
		// but manual adjustments (e.g. via a play space mover) will not touch geometry.
		if (quadCount != ctx.chaperone.geometry.size())
		{
			ApplyChaperoneBounds();
		}
	}
}

void StartCalibration()
{
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
	calibration.Clear();
}

void StartContinuousCalibration()
{
	CalCtx.hasAppliedCalibrationResult = false;
	AssignTargets();
	StartCalibration();
	CalCtx.state = CalibrationState::Continuous;

	// Set relative transformation for continuous calibration
	calibration.setRelativeTransformation(CalCtx.refToTargetPose, CalCtx.relativePosCalibrated);
	calibration.lockRelativePosition = CalCtx.lockRelativePosition;

	if (CalCtx.lockRelativePosition) {
		CalCtx.Log("Relative position locked\n");
	}
	else {
		CalCtx.Log("Collecting initial samples...\n");
	}

	// Note: Metrics::WriteLogAnnotation not ported to Linux version
}

void EndContinuousCalibration()
{
	CalCtx.state = CalibrationState::None;
	CalCtx.relativePosCalibrated = false;
	SaveProfile(CalCtx);
	CalCtx.Log("Continuous calibration stopped, profile saved\n");
	// Note: Metrics::WriteLogAnnotation not ported to Linux version
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem())
		return;

	auto &ctx = CalCtx;
	if ((time - ctx.timeLastTick) < 0.05)
		return;

	// Handle continuous calibration clearing log messages
	if (ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby) {
		ctx.ClearLogOnMessage();

		if (CalCtx.requireTriggerPressToApply && (time - ctx.timeLastAssign) > 10) {
			// Rescan devices every 10 seconds if we are using controller data
			ctx.timeLastAssign = time;
			AssignTargets();
		}
	}

	ctx.timeLastTick = time;

	// Read poses from shared memory (driver poses with proper transforms!)
	shmem.ReadNewPoses([&](const protocol::DriverPoseShmem::AugmentedPose& augmented_pose) {
		if (augmented_pose.deviceId >= 0 && augmented_pose.deviceId <= vr::k_unMaxTrackedDeviceCount) {
			ctx.driverPoses[augmented_pose.deviceId] = augmented_pose.pose;
		}
	});

	// Also read from VR API as fallback (for HMD tracking check)
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, ctx.devicePoses, vr::k_unMaxTrackedDeviceCount);

	// Check for non-updating HMD tracking (Quest out of bounds, etc.)
	auto &hmdPose = ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
	if (hmdPose.bPoseIsValid) {
		auto p = hmdPose.mDeviceToAbsoluteTracking.m;
		float px = p[0][3], py = p[1][3], pz = p[2][3];

		// Skip if HMD is at origin or hasn't moved since last tick
		if ((px == 0.0f && py == 0.0f && pz == 0.0f) ||
		    (ctx.xprev == px && ctx.yprev == py && ctx.zprev == pz)) {
			// HMD tracking didn't update, skip this tick to avoid bad samples
			return;
		}

		ctx.xprev = px;
		ctx.yprev = py;
		ctx.zprev = pz;
	}

	// DIAGNOSTIC: Track device ID changes during continuous calibration
	static uint32_t lastReferenceID = ctx.referenceID;
	static uint32_t lastTargetID = ctx.targetID;
	static bool deviceIDsInitialized = false;

	if (ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::Rotation || ctx.state == CalibrationState::Translation)
	{
		if (!deviceIDsInitialized)
		{
			lastReferenceID = ctx.referenceID;
			lastTargetID = ctx.targetID;
			deviceIDsInitialized = true;

			char buf[256];
			snprintf(buf, sizeof buf, "[DIAGNOSTIC] Calibration started with Reference ID: %d, Target ID: %d\n", ctx.referenceID, ctx.targetID);
			CalCtx.Log(buf);
		}
		else if (lastReferenceID != ctx.referenceID || lastTargetID != ctx.targetID)
		{
			char buf[512];
			snprintf(buf, sizeof buf, "[DIAGNOSTIC WARNING] Device IDs changed during calibration!\n  Old: Ref=%d, Target=%d\n  New: Ref=%d, Target=%d\n  Sample count: %zd\n",
				lastReferenceID, lastTargetID, ctx.referenceID, ctx.targetID, calibration.SampleCount());
			CalCtx.Log(buf);

			lastReferenceID = ctx.referenceID;
			lastTargetID = ctx.targetID;
		}
	}
	else if (ctx.state == CalibrationState::None)
	{
		deviceIDsInitialized = false;
	}

	// Handle continuous calibration states
	if (ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby)
	{
		if ((time - ctx.timeLastScan) >= 1.0)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
	}

	if (ctx.state == CalibrationState::None)
	{
		ctx.wantedUpdateInterval = 1.0;

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = 0.1;

		if ((time - ctx.timeLastScan) >= 0.1)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Begin)
	{
		bool ok = true;

		char referenceSerial[256], targetSerial[256];
		vr::VRSystem()->GetStringTrackedDeviceProperty(ctx.referenceID, vr::Prop_SerialNumber_String, referenceSerial, 256);
		vr::VRSystem()->GetStringTrackedDeviceProperty(ctx.targetID, vr::Prop_SerialNumber_String, targetSerial, 256);

		char buf[256];
		snprintf(buf, sizeof buf, "Reference device ID: %d, serial: %s\n", ctx.referenceID, referenceSerial);
		CalCtx.Log(buf);
		snprintf(buf, sizeof buf, "Target device ID: %d, serial %s\n", ctx.targetID, targetSerial);
		CalCtx.Log(buf);

		if (ctx.referenceID == -1)
		{
			CalCtx.Log("Missing reference device\n"); ok = false;
		}
		else if (!ctx.devicePoses[ctx.referenceID].bPoseIsValid)
		{
			CalCtx.Log("Reference device is not tracking\n"); ok = false;
		}

		if (ctx.targetID == -1)
		{
			CalCtx.Log("Missing target device\n"); ok = false;
		}
		else if (!ctx.devicePoses[ctx.targetID].bPoseIsValid)
		{
			CalCtx.Log("Target device is not tracking\n"); ok = false;
		}

		// Check jitter to ensure tracking quality
		if (calibration.ReferenceJitter() > ctx.jitterThreshold) {
			CalCtx.Log("Reference device tracking is too jittery\n"); ok = false;
		}
		if (calibration.TargetJitter() > ctx.jitterThreshold) {
			CalCtx.Log("Target device tracking is too jittery\n"); ok = false;
		}

		if (!ok)
		{
			if (ctx.state != CalibrationState::Continuous) {
				ctx.state = CalibrationState::None;
				CalCtx.Log("Aborting calibration!\n");
			}
			return;
		}

		ResetAndDisableOffsets(ctx.targetID);
		ctx.state = CalibrationState::Rotation;
		ctx.wantedUpdateInterval = 0.0;

		CalCtx.Log("Starting calibration...\n");
		return;
	}

	auto sample = CollectSample(ctx);
	if (!sample.valid)
	{
		return;
	}

	// Push sample to CalibrationCalc for continuous mode
	calibration.PushSample(sample);

	static std::vector<Sample> samples;
	samples.push_back(sample);

	CalCtx.Progress(calibration.SampleCount(), CalCtx.SampleCount());

	if (calibration.SampleCount() < CalCtx.SampleCount())
	{
		return;
	}

	// Drop excess samples
	while (calibration.SampleCount() > CalCtx.SampleCount())
	{
		calibration.ShiftSample();
	}

	if (samples.size() >= CalCtx.SampleCount())
	{
		CalCtx.Log("\n");

		bool lerp = false;
		bool calibrationSuccess = false;

		// Use incremental computation for continuous mode, one-shot for manual
		if (ctx.state == CalibrationState::Continuous)
		{
			// Set calibration flags before computing
			calibration.enableStaticRecalibration = ctx.enableStaticRecalibration;
			calibration.lockRelativePosition = ctx.lockRelativePosition;
			calibrationSuccess = calibration.ComputeIncremental(lerp, ctx.continuousCalibrationThreshold, ctx.maxRelativeErrorThreshold, ctx.ignoreOutliers);
		}
		else
		{
			calibrationSuccess = calibration.ComputeOneshot(ctx.ignoreOutliers);
		}

		if (calibrationSuccess && calibration.isValid())
		{
			// Store calibration results
			ctx.calibratedRotation = calibration.EulerRotation();
			ctx.calibratedTranslation = calibration.Transformation().translation() * 100.0; // Convert to cm
			ctx.refToTargetPose = calibration.RelativeTransformation();  // CRITICAL for continuous calibration!
			ctx.relativePosCalibrated = calibration.isRelativeTransformationCalibrated();  // CRITICAL!

			ctx.validProfile = true;
			SaveProfile(ctx);  // Save profile after every update

			// Apply calibration to all devices with lerp/quash flags
			ScanAndApplyProfile(ctx);  // This sets lerp=true for Continuous mode!

			CalCtx.hasAppliedCalibrationResult = true;

			if (ctx.state == CalibrationState::Continuous)
			{
				CalCtx.Log("Continuous calibration updated\n");
				// Drop some samples to make room for new ones
				size_t dropSamples = CalCtx.SampleCount() / 10;
				for (size_t i = 0; i < dropSamples && !samples.empty(); i++)
				{
					samples.erase(samples.begin());
					calibration.ShiftSample();
				}
			}
			else
			{
				CalCtx.Log("Finished calibration, profile saved\n");
				ctx.state = CalibrationState::None;
				samples.clear();
				calibration.Clear();
			}
		}
		else
		{
			if (ctx.state != CalibrationState::Continuous)
			{
				CalCtx.Log("Calibration failed!\n");
				ctx.state = CalibrationState::None;
				samples.clear();
				calibration.Clear();
			}
		}
	}
}

void LoadChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

	CalCtx.chaperone.geometry.resize(quadCount);
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], &quadCount);
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->GetWorkingPlayAreaSize(&CalCtx.chaperone.playSpaceSize.v[0], &CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.chaperone.valid = true;
}

void ApplyChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();
	vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], CalCtx.chaperone.geometry.size());
	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}
