# Auto-Align + Smart Shooter Integration Guide
## FRC 2026 — CTRE Swerve + PhotonVision

---

## Files Delivered

| File | Purpose |
|------|---------|
| `ShooterConstants.java` | Single source of truth for all geometry, PID gains, physics constants |
| `ShotCalculator.java` | Ballistics engine — ports your `frc_v0_calculator.html` to Java |
| `HubAlignController.java` | Stateful PD + filter + slew controller, odometry-based (no raw-camera jitter) |
| `DriveToHubAndShootCommand.java` | Top-level command + `IShooterSubsystem` interface |
| `VisionSubsystem.java` | Updated vision subsystem with hub-specific target tracking |

---

## Step 1 — Edit `ShooterConstants.java`

These are the values you **must** update before testing:

```java
// Hub position for 2026 — update when field layout is released
public static final Translation2d HUB_CENTER = new Translation2d(8.270, 4.105);
public static final double HUB_TARGET_HEIGHT_METERS = 2.64;

// Hub AprilTag IDs — check the 2026 game manual
public static final int[] HUB_APRIL_TAG_IDS = {4, 19};

// Your robot's shooter exit height
public static final double SHOOTER_HEIGHT_METERS = 0.52;

// How far from the hub you want to stand
public static final double DESIRED_DISTANCE_METERS = 2.5;

// Your flywheel wheel diameter (inches) and gear ratio
public static final double WHEEL_DIAMETER_INCHES = 4.0;
public static final double GEAR_RATIO = 1.0;
public static final double EFFICIENCY = 0.85;
```

---

## Step 2 — Implement `IShooterSubsystem` in your Shooter

```java
public class ShooterSubsystem extends SubsystemBase
        implements DriveToHubAndShootCommand.IShooterSubsystem {

    private static final double RPM_TOLERANCE_PCT = 0.03;   // 3%
    private static final double ANGLE_TOLERANCE_DEG = 1.0;

    private double m_targetRPM   = 0;
    private double m_targetAngle = 0;

    @Override
    public void setFlywheelRPM(double rpm) {
        m_targetRPM = rpm;
        // Set your motor controller velocity setpoint here.
        // Example (TalonFX Phoenix 6 velocity control):
        // m_flywheelMotor.setControl(new VelocityVoltage(rpm / 60.0 * GEAR_RATIO));
    }

    @Override
    public void setLaunchAngleDeg(double degrees) {
        m_targetAngle = degrees;
        // Set your pivot / hood position.
        // Example: m_pivot.setControl(new PositionVoltage(degreesToRotations(degrees)));
    }

    @Override
    public void idleFlywheel() {
        m_targetRPM = 500; // gentle idle
        // m_flywheelMotor.setControl(new VelocityVoltage(500.0 / 60.0 * GEAR_RATIO));
    }

    @Override
    public void shoot() {
        // Activate your indexer / kicker
        // m_indexer.set(ControlMode.PercentOutput, 1.0);
    }

    @Override
    public boolean isAtTargetRPM(double targetRPM) {
        double currentRPM = /* read encoder */ 0;
        return Math.abs(currentRPM - targetRPM) < targetRPM * RPM_TOLERANCE_PCT;
    }

    @Override
    public boolean isAtTargetAngle(double targetDeg) {
        double currentDeg = /* read absolute encoder */ 0;
        return Math.abs(currentDeg - targetDeg) < ANGLE_TOLERANCE_DEG;
    }

    @Override
    public edu.wpi.first.wpilibj2.command.Subsystem asSubsystem() {
        return this;
    }
}
```

---

## Step 3 — Wire the Command in `RobotContainer`

```java
// Instantiate
private final VisionSubsystem m_vision = new VisionSubsystem(
    new String[]{"FrontCamera", "BackCamera"},
    m_drivetrain);

private final ShooterSubsystem m_shooter = new ShooterSubsystem();

// Bind to a button (hold to align + shoot)
new JoystickButton(m_driverXbox, XboxController.Button.kA.value)
    .whileTrue(new DriveToHubAndShootCommand(m_drivetrain, m_vision, m_shooter));
```

---

## Step 4 — PID Tuning Order

Tune one axis at a time with the robot on blocks:

### Rotation first
1. Set `ROTATION_kP = 1.0`, `ROTATION_kD = 0.0`.
2. Point robot 45° away from hub, trigger command.
3. Increase kP until it oscillates, then back off 30%.
4. Add kD (~10% of kP) to damp overshoot.

### Distance second
1. Set `DRIVE_kP = 1.0`, `DRIVE_kD = 0.0`.
2. Place robot 1 m too far from hub.
3. Same process — increase until oscillation, back off, add D.

### Strafe last
1. Offset robot 0.5 m laterally from the approach line.
2. Same tuning loop.

### Anti-jitter knobs (if you see oscillation at setpoint)
- Increase `OUTPUT_FILTER_ALPHA` (0.1 → 0.3) for more smoothing.
- Decrease `DRIVE_MAX_ACCEL_MPS2` to reduce jerk.
- Widen `VELOCITY_DEADBAND` slightly if chatter persists at rest.

---

## Step 5 — Shot Calculator Validation

Your HTML calculator and `ShotCalculator.java` use the same formula.
Quick sanity check in a unit test or in `Robot.java`:

```java
ShotResult r = ShotCalculator.calculate(2.5); // 2.5 m to hub
System.out.println(r); // Should print something like:
// ShotResult{θ=52.0°, v0=7.34m/s, RPM=4120, entry=41.2°, t=0.412s, d=2.50m, score=0.731}
```

Compare the angle and v0 values against your HTML tool at the same distance.
They should match to within ±0.1°.

---

## Architecture Diagram

```
PhotonVision cameras
        │  (50 Hz raw detections)
        ▼
VisionSubsystem.periodic()
  └─ addVisionMeasurement() ──► SwerveDrivetrainOdometry (Kalman filter)
                                        │
                           getState().Pose (fused, low-noise)
                                        │
                    ┌───────────────────┘
                    │
         HubAlignController.update()
           ├─ distance PD  ──► drive speed (m/s)
           ├─ lateral PD   ──► strafe speed (m/s)
           └─ heading PD   ──► omega (rad/s)
                    │
                    ▼
         CommandSwerveDrivetrain
           (FieldCentric SwerveRequest)


         ShotCalculator.calculate(distance)
           └─ physics model from frc_v0_calculator.html
                    │
                    ▼
         ShooterSubsystem
           ├─ setLaunchAngleDeg()
           ├─ setFlywheelRPM()
           └─ shoot()  ◄── fires only when ALL three gates pass:
                             • align.readyToShoot  (dist + yaw)
                             • isAtTargetRPM()
                             • isAtTargetAngle()
```

---

## Notes on Jitter Prevention

| Source of jitter | How it's eliminated |
|-----------------|---------------------|
| Raw camera yaw fed directly to drive motor | **Never done.** Drive outputs derive from fused odometry pose only. Camera data enters only through the Kalman filter in `addVisionMeasurement()`. |
| PID derivative kick on new setpoint | Derivative computed on **error**, not setpoint — no spike when target updates. |
| High-frequency noise in pose estimate | Low-pass filter on all three PD outputs (`OUTPUT_FILTER_ALPHA`). |
| Wheel jerk from sudden commands | Slew-rate limiters on all three output axes. |
| Micro-oscillation at setpoint | Velocity deadband zeros commands below `VELOCITY_DEADBAND`. |
| Repeated pose estimation calls per loop | `updateCameraPoses()` reuses results already computed in `periodic()`. |