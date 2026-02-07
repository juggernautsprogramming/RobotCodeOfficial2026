# TODO List for Updating VisionSubsystem.java and ObjectCamera.java for WPILib 2026 and PhotonVision 2026

- [x] Edit Constants.java: Add Resolution class
- [x] Edit Constants.java: Add VisionHardware class with camera constants
- [x] Edit Constants.java: Add Field class with updated AprilTagFieldLayout for k2025Reefscape
- [x] Edit VisionSubsystem.java: Add missing imports (Distance, Angle, Logger)
- [x] Edit VisionSubsystem.java: Update AprilTagFields to k2025Reefscape
- [x] Edit VisionSubsystem.java: Fix getObjectLocation to use actual distance
- [x] Edit VisionSubsystem.java: Initialize m_cameraNotifier
- [x] Edit VisionSubsystem.java: Add sim update in simulationPeriodic
- [x] Edit VisionSubsystem.java: Remove/comment AprilTagCamera references
- [x] Edit ObjectCamera.java: Add missing imports (Distance, Angle)
- [x] Test compilation
- [ ] Verify vision functionality in sim/real robot
