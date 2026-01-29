^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package l3cam_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2024-07-10)
-----------
- Updated threading in streaming nodes.
- Added simulator functionality to use L3CamSimulator.
- Added processing of polarimetric camera.
- Added streaming of 2D and 3D detections.
- Added nodes for streaming compressed images.
- Added node for streaming tiny point clouds (custom compression message).
- Better debugging for setting socket receive buffer size.
- Better debugging for lost UDP packets.


1.0.2 (2024-07-10)
-----------
- thermalTypes changed for newThermalTypes.
- LibL3CamStatus status types with '_status' suffix to avoid errors.
- libl3cam 0.1.18 new functionalities.
- Added missing namespace prefix on messages.
- Fixed no error on lidar configuration bug.
- Bias value left and right bug.
- Added econ wide implementation

1.0.1 (2024-03-15)
-----------
- Sensors header stamps with unix epoch timestamp
- Allied cameras stream in YUV
- Topic for thermal float (pixels with temperature values)
- New libl3cam thermal camera functionalities
- Device info service
- Changed sensors header stamps to unix epoch timestamp
- Allied cameras stream in YUV
- Added topic for thermal float (pixels with temperature values)
- New libl3cam thermal camera functionalities
- Added device info service

1.0.0 (2023-12-11)
-----------
* Updated package to libL3Cam 0.1.15
* Contributors: Adrià Subirana

0.0.3 (2023-03-23)
-----------
* Added allied Wide and Narrow cameras
* Implemented default parameters
* Bug fixes
* Disable dynamic network configuration
* Contributors: Adrià Subirana

0.0.2 (2023-02-06)
-----------
* Args for launch files
* Licenses
* README
* Library externalization
* Contributors: Adrià Subirana

0.0.1 (2023-01-16)
-----------
* changelog
* gitignore
* first commit
* Contributors: Adrià Subirana
