# OpenVR Space Calibrator - Linux with Continuous Calibration

A Linux port of OpenVR Space Calibrator with full continuous calibration support. This tool helps you use tracked VR devices from different tracking systems together (e.g., ALVR headset with Lighthouse trackers).

**Key Features:**
- Full continuous calibration support for smooth, real-time adjustments
- Works with ALVR, Lighthouse tracking, and other mixed VR setups
- Automatic calibration persistence across sessions
- Compatible with SteamVR on Linux

For Monado/WiVRn users, see [MoToC](https://github.com/galister/motoc) instead.

## Credits

- Original code by [pushrax](https://github.com/pushrax)
- Base Linux fork by [gyroninja](https://github.com/gyroninja)
- Hook logic from [AngriestSCV](https://github.com/AngriestSCV)'s fork
- Additional features from [bd_](https://github.com/bdunderscore)'s fork
- **Continuous Calibration Linux port and fixes by [Stavdel](https://github.com/Stavdel)**

## Building

The following packages are required to build:

* `cmake`
* `glfw`
* `eigen`
* `openvr`

Arch Linux:

* `pacman -S cmake glfw-x11 eigen openvr`

Debian 11 / Ubuntu 22.10:

* `sudo apt install cmake libeigen3-dev libglfw3-dev libopenvr-dev`

```bash
git clone https://github.com/Stavdel/OpenVR-SpaceCalibrator.git
cd OpenVR-SpaceCalibrator
mkdir build
cd build
../install.sh
```

## Running

**If using ALVR**: 

- Launch ALVR Dashboard
- On the `Settings` tab:
  - Under `Headset`, set both `Position Recentering Mode` and `Rotation Recentering Mode` to `None`
  - Under `SteamVR Launcher`, set `Driver Launch Action` to `None`
- On the `Installation` tab, click `Register ALVR Driver`

If you used the `install.sh` script, the driver will be loaded by SteamVR, but the companion software must be started separately every time.

For best results, I recommend doing the calibration:

- in SteamVR void/home, before launching any titles, especially if you have high latency or low frame rate
- with all trackers already connected. I've seen some glitches when connecting new trackers.

### Step-by-step Guide to Calibrate using ALVR

#### Verify settings

- Installation tab: ALVR should be listed under `Registered Drivers`. If not, press `Register ALVR driver`
- Settings tab -> Steamvr launcher -> Driver launch action: Unregister ALVR at shutdown. This prevents OpenVR Space Calibrator from unregistering by alvr when closing steamvr.

#### If you want to use Lighthouse-tracked controllers with ALVR (not needed if you just want trackers)

- Settings/Headset tab -> `Controllers` -> `Tracked`: off

#### Note to Pico users

Pico users might also want to increase the sleep timeout to 5 minutes or higher, as you will lose calibration if the headset goes to sleep: [Link](https://www.reddit.com/r/PICO_VR/comments/zmspi9/i_managed_to_turn_off_the_pico_4s_sleep_mode_by/) \
(Will not lose calibration on screen-off, as long as you disabled recentering as per above.)

To prevent issues with constant recentering on Pico, please change the following in ALVR dashboard:

* Settings -> Headset -> Position recentring mode: Disabled

* Settings -> Headset -> Rotation recentring mode: Disabled

### Steps:

If you have ALVR open at this point, close it as well as SteamVR.

- Launch SteamVR and ALVR.
- Start the `openvr-spacecalibrator` companion software.
- Connect your headset via ALVR.
- Turn on one lighthouse-tracked device.
- Select these settings on the Space Calibrator window (in VR or on destkop)
  - Calibration Speed: `Slow` or `Very Slow`
  - Reference Space: The top option, which should be the headset
  - Target space: Select the lighthouse device that you'll be using
- Click `Start Calibration` while holding the lighthouse device firmly to your headset.
  - The lighthouse device shouldn't wiggle, change position or orientation relative to the headset while calibration.
- Walk around your play area while still holding the lighthouse device firmly.
- Once calibration is done, power on your other lighthouse devices, they should be good to go.

Base stations may show up in the wrong places, this issue can be ignored.

## Known shortcomings brought to you by SteamVR

There are a few issues that can pop up. Luckily, these can all be identified before calibration is started and don't pose an issue later during your session.

- Disconnected dongles cannot be reconnected without a SteamVR restart.
  - Ensure that all dongles have a stable USB link so that they won't momentarily disconnect.
- Tracker turns off and/or stuck blue and won't turn green.
  - This is likely because your dongle got disconnected momentarily.
- One or more trackers just don't show up in SteamVR, even though they're solid green.
  - Verify that all trackers show up in SteamVR before you do start a title or calibrate, if anything's amiss, restart SteamVR
- Head tracking is weird, image rotates differently than your headset, ALVR graph showing red mountains.
  - Restart SteamVR and make sure to connect the headset first, before you power on any lighthouse devices.
  - If you saw an image on the headset at least once since starting SteamVR, it's safe to turn on lighthouse devices (even if the headset is no longer connected).

---

## Device Compatibility

This tool aligns multiple tracking systems with a quick calibration step. Results vary depending on your setup:

- **ALVR/Quest + Lighthouse trackers**: Works well for stationary activities (e.g. Beat Saber). Walking around extensively may cause drift between tracking systems. Use **Slow** or **Very Slow** calibration modes for best results with wireless streaming.
- **Windows MR/other SLAM HMDs + Lighthouse devices**: Similar performance to ALVR setup - excellent for limited movement, some drift with extensive roomscale.
- **Rift CV1 + Vive devices**: Generally works very well with Vive trackers (v2 recommended), controllers, and Index controllers.

## How It Works

Once calibrated, Space Calibrator works in the background to keep your devices aligned. Everything is automated except the initial calibration process.

### Continuous Calibration

This version includes full continuous calibration support, which provides:
- **Smooth real-time adjustments** - Devices interpolate smoothly into position instead of snapping
- **Automatic refinement** - Calibration continuously improves as you move around
- **Persistent state** - Settings are saved and restored across sessions
- **Adaptive speed** - Calibration speed adjusts based on movement magnitude

To use continuous calibration:
1. Complete an initial calibration (see below)
2. Click **"Start Continuous Calibration"** in the UI
3. Move around normally - the system will refine calibration automatically

### Initial Calibration

As part of first time setup, or when you make a change to your space (e.g. move a sensor), and occasionally as the calibration drifts over time (consumer VR tracking isn't perfectly stable), you'll need to run a calibration:

1. Copy the chaperone/guardian bounds from your HMD's play space. This doesn't need to be run if your HMD's play space hasn't changed since last time you copied it. __Example:__ if you're using the Rift with Vive trackers and you bump a Vive lighthouse, or if the calibration has just drifted a little, you likely don't need to run this step, but if you bump an Oculus sensor you will (after running Oculus guardian setup again).
   
   1. Run SteamVR, with only devices from your HMD's tracking system powered on. __Example:__ for Rift with Vive trackers, don't turn on the trackers yet.
   2. Confirm your chaperone/guardian is set up with the walls in the right place. If you change it later, you need to run step again.
   3. Open SPACE CAL in the SteamVR dashboard overlay.
   4. Click `Copy Chaperone Bounds to profile`

2. Calibrate devices.
   
   1. Open SteamVR if you haven't already. Turn on some or all your devices.
   2. Open SPACE CAL in the SteamVR dashboard overlay.
   3. Select one device from the reference space on the left and one device from the target space on the right. If you turned on multiple devices from one space and can't tell which one is selected, click "Identify selected devices" to blink an LED or vibrate it. __Example:__ for Rift with Vive trackers, you'll see the Touch controllers on the left, and Vive trackers on the right. __Pro tip:__ if you turn on just one Vive tracker, you don't have to figure out which one is selected.
   4. Hold these two devices in one hand, like they're glued together. If they slip, calibration won't work as well.
   5. Click `Start Calibration`
   6. Move and rotate your hand around slowly a few times, like you're calibrating the compass on your phone. You want to sample as many orientations as possible.
   7. Done! A profile will be saved automatically. If you haven't already, turn on all your devices. Space Calibrator will automatically apply the calibration to devices as they turn on.

### Calibration outside of VR

You can calibrate without using the dashboard overlay by unminimizing Space Calibrator after opening SteamVR. This is required if you're calibrating for a lone HMD without any devices in its tracking system.

## Support

For bug reports, feature requests, or issues, please use the [GitHub Issues](https://github.com/Stavdel/OpenVR-SpaceCalibrator/issues) page.
