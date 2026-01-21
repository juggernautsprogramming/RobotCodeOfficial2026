# TODO: Add Nudge Control with D-pad on Xbox Controller

## Steps to Complete:
- [x] Define a nudge speed constant (e.g., 30% of MaxSpeed) in RobotContainer.java
- [x] Add D-pad bindings in configureBindings() method:
  - [x] Bind povUp() to forward nudge (positive X velocity)
  - [x] Bind povDown() to backward nudge (negative X velocity)
  - [x] Bind povLeft() to left nudge (negative Y velocity)
  - [x] Bind povRight() to right nudge (positive Y velocity)
- [ ] Test the implementation (followup step)

## Dependent Files:
- src/main/java/frc/robot/RobotContainer.java

## Followup Steps:
- Verify the changes compile and run without errors
- Test on the robot to ensure nudge control works as expected
