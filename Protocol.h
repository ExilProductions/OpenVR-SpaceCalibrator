#pragma once

#include <cstdint>
#include <atomic>
#include <functional>
#include <cstring>

// Linux-specific includes for shared memory
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <time.h>

#ifndef _OPENVR_API
#include <openvr_driver.h>
#endif

#define OPENVR_SPACECALIBRATOR_PIPE_NAME "/tmp/OpenVRSpaceCalibratorDriver.sock"
#define OPENVR_SPACECALIBRATOR_SHMEM_NAME "/OpenVRSpaceCalibratorPoseMemory"

// When included in overlay (not driver), define DriverPose_t ourselves
#ifdef _OPENVR_API
namespace vr {
	struct DriverPose_t
	{
		double poseTimeOffset;
		vr::HmdQuaternion_t qWorldFromDriverRotation;
		double vecWorldFromDriverTranslation[3];
		vr::HmdQuaternion_t qDriverFromHeadRotation;
		double vecDriverFromHeadTranslation[3];
		double vecPosition[3];
		double vecVelocity[3];
		double vecAcceleration[3];
		vr::HmdQuaternion_t qRotation;
		double vecAngularVelocity[3];
		double vecAngularAcceleration[3];
		ETrackingResult result;
		bool poseIsValid;
		bool willDriftInYaw;
		bool shouldApplyHeadModel;
		bool deviceIsConnected;
	};
}
#endif

namespace protocol
{
	const uint32_t Version = 4;

	enum RequestType
	{
		RequestInvalid,
		RequestHandshake,
		RequestSetDeviceTransform,
		RequestSetAlignmentSpeedParams,
		RequestDebugOffset
	};

	enum ResponseType
	{
		ResponseInvalid,
		ResponseHandshake,
		ResponseSuccess,
	};

	struct Protocol
	{
		uint32_t version = Version;
	};

	struct AlignmentSpeedParams
	{
		/**
		 * The threshold at which we adjust the alignment speed based on the position offset
		 * between current and target calibrations. Generally, we increase the speed if we go
		 * above small/large, and decrease it only once it's under tiny.
		 *
		 * These values are expressed as distance squared
		 */
		double thr_trans_tiny, thr_trans_small, thr_trans_large;

		/**
		 * Similar thresholds for rotation offsets, in radians
		 */
		double thr_rot_tiny, thr_rot_small, thr_rot_large;

		/**
		 * The speed of alignment, expressed as a lerp/slerp factor. 1 will blend most of the way in <1 second.
		 * (We actually do a lerp(s * delta_t) where s is the speed factor here)
		 */
		double align_speed_tiny, align_speed_small, align_speed_large;
	};

	struct SetDeviceTransform
	{
		uint32_t openVRID;
		bool enabled;
		bool updateTranslation;
		bool updateRotation;
		bool updateScale;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;
		bool lerp;
		bool quash;

		SetDeviceTransform(uint32_t id, bool enabled) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(false), translation({}), rotation({1,0,0,0}), scale(1), lerp(false), quash(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(false), updateScale(false), translation(translation), rotation({1,0,0,0}), scale(1), lerp(false), quash(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(true), updateScale(false), translation({}), rotation(rotation), scale(1), lerp(false), quash(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(true), translation({}), rotation({1,0,0,0}), scale(scale), lerp(false), quash(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(false), translation(translation), rotation(rotation), scale(1), lerp(false), quash(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(true), translation(translation), rotation(rotation), scale(scale), lerp(false), quash(false) { }
	};

	struct Request
	{
		RequestType type;

		union {
			SetDeviceTransform setDeviceTransform;
			AlignmentSpeedParams setAlignmentSpeedParams;
		};

		Request() : type(RequestInvalid), setAlignmentSpeedParams({}) { }
		Request(RequestType type) : type(type), setAlignmentSpeedParams({}) { }
		Request(AlignmentSpeedParams params) : type(RequestType::RequestSetAlignmentSpeedParams), setAlignmentSpeedParams(params) {}
	};

	struct Response
	{
		ResponseType type;

		union {
			Protocol protocol;
		};

		Response() : type(ResponseInvalid) { }
		Response(ResponseType type) : type(type) { }
	};

	// Shared memory for real-time pose streaming from driver to overlay
	class DriverPoseShmem {
	public:
		struct AugmentedPose {
			timespec sample_time;  // Linux equivalent of LARGE_INTEGER
			int deviceId;
			vr::DriverPose_t pose;
		};

	private:
		static const uint32_t BUFFERED_SAMPLES = 64 * 1024;

		struct ShmemData {
			std::atomic<uint64_t> index;
			AugmentedPose poses[BUFFERED_SAMPLES];
		};

	private:
		int fd;
		ShmemData* pData;
		uint64_t cursor;
		AugmentedPose lastPose[vr::k_unMaxTrackedDeviceCount];

	public:
		operator bool() const {
			return pData != nullptr;
		}

		bool operator!() const {
			return pData == nullptr;
		}

		DriverPoseShmem() {
			fd = -1;
			pData = nullptr;
			cursor = 0;
			memset(lastPose, 0, sizeof(lastPose));
		}

		~DriverPoseShmem() {
			Close();
		}

		void Close() {
			if (pData) {
				munmap(pData, sizeof(ShmemData));
				pData = nullptr;
			}
			if (fd >= 0) {
				close(fd);
				fd = -1;
			}
		}

		bool Create(const char* segment_name) {
			Close();

			// Create shared memory object
			fd = shm_open(segment_name, O_CREAT | O_RDWR, 0666);
			if (fd < 0) {
				return false;
			}

			// Set size
			if (ftruncate(fd, sizeof(ShmemData)) < 0) {
				close(fd);
				fd = -1;
				return false;
			}

			// Map it
			pData = reinterpret_cast<ShmemData*>(mmap(
				nullptr,
				sizeof(ShmemData),
				PROT_READ | PROT_WRITE,
				MAP_SHARED,
				fd,
				0
			));

			if (pData == MAP_FAILED) {
				close(fd);
				fd = -1;
				pData = nullptr;
				return false;
			}

			// Initialize
			pData->index = 0;

			return true;
		}

		bool Open(const char* segment_name) {
			Close();

			// Open existing shared memory
			fd = shm_open(segment_name, O_RDWR, 0666);
			if (fd < 0) {
				return false;
			}

			// Map it
			pData = reinterpret_cast<ShmemData*>(mmap(
				nullptr,
				sizeof(ShmemData),
				PROT_READ | PROT_WRITE,
				MAP_SHARED,
				fd,
				0
			));

			if (pData == MAP_FAILED) {
				close(fd);
				fd = -1;
				pData = nullptr;
				return false;
			}

			cursor = pData->index;
			return true;
		}

		void WritePose(int deviceId, const vr::DriverPose_t& pose) {
			if (!pData) return;

			uint64_t writeIndex = pData->index.fetch_add(1);
			uint64_t slot = writeIndex % BUFFERED_SAMPLES;

			AugmentedPose& aug = pData->poses[slot];
			clock_gettime(CLOCK_MONOTONIC, &aug.sample_time);
			aug.deviceId = deviceId;
			aug.pose = pose;
		}

		template<typename F>
		void ReadNewPoses(F callback) {
			if (!pData) return;

			uint64_t latestIndex = pData->index.load();

			// Catch up if we're too far behind
			if (latestIndex > cursor + BUFFERED_SAMPLES) {
				cursor = latestIndex - BUFFERED_SAMPLES;
			}

			// Read all new poses
			while (cursor < latestIndex) {
				uint64_t slot = cursor % BUFFERED_SAMPLES;
				const AugmentedPose& aug = pData->poses[slot];

				if (aug.deviceId >= 0 && aug.deviceId < vr::k_unMaxTrackedDeviceCount) {
					// Only callback if pose is newer than last one for this device
					auto& last = lastPose[aug.deviceId];
					if (aug.sample_time.tv_sec > last.sample_time.tv_sec ||
					    (aug.sample_time.tv_sec == last.sample_time.tv_sec &&
					     aug.sample_time.tv_nsec > last.sample_time.tv_nsec)) {
						lastPose[aug.deviceId] = aug;
						callback(aug);
					}
				}

				cursor++;
			}
		}
	};
}