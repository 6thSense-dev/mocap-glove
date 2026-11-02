# 6thSense magnetic MoCap glove

STM32G474 diagnostic/streaming firmware and local magnetic fingertip tracking viewer.

## Run locally

Install Python dependencies with `python3 -m pip install -r requirements.txt`, then run `python3 control-panel/server.py` and open http://127.0.0.1:8775/hand.

Read START-HERE.txt before powered tests. Firmware build instructions are in firmware-app/BUILD.txt. The viewer does not automatically flash hardware.

## Saved scope and limitations

Includes firmware source, local viewer, calibration algorithms, automated tests and required vendor assets. Hardware-specific calibration profiles, recordings, flash backups and build outputs remain on the development machine and are intentionally excluded. An empty checkout requires its own calibration before calibrated tracking can be reproduced. Browser display preferences are local to the browser.

Jitter filtering, continuous mirror-branch tracking and reversible per-finger Z display correction are included. Z reversal is a display correction, not physical calibration. Magnetic mirror ambiguity and absolute accuracy are not established by software tests. IMU translation is experimental and drifts; CAD joint angles are inferred robot retargeting, not independently measured finger joints.

## Third-party assets

The Aero Hand Open meshes and URDF originate from Chestnut Robotics (formerly TetherIA), commit 7bb2f23485a7d440960c21046da642de132b7745, under CC BY-NC-SA 4.0. Preserve the license, attribution and provenance in control-panel/vendor/aero-hand/. The current integration is for the user-confirmed noncommercial use. This is not a blanket license for commercial use of those assets.

Three.js and ST firmware dependencies retain their upstream license files. No blanket license is granted over the entire repository.

## Tests

`python3 -m unittest discover -s control-panel -p 'test_*.py'`

`node control-panel/test-hand-z-reflection.cjs`

`node control-panel/test-aero-kinematics.cjs`

`node control-panel/test-hand-view-frame.cjs`
