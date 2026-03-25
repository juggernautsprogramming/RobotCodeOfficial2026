package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC; // Phoenix Pro — requires activated license
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    // ── Pivot hardware ────────────────────────────────────────────────────────
    private final TalonFX m_pivotLeader;
    private final TalonFX m_pivotFollower;
    private final MotionMagicVoltage m_mmRequest =
        new MotionMagicVoltage(0).withEnableFOC(true);

    // ── Flywheel hardware ─────────────────────────────────────────────────────
    private final TalonFX m_flywheelLeader;
    private final TalonFX m_flywheelFollower;

    // VelocityTorqueCurrentFOC: Phoenix Pro only. Runs the closed-loop on the
    // motor's 1 kHz update cycle and is immune to battery voltage sag.
    private final VelocityTorqueCurrentFOC m_velocityRequest =
        new VelocityTorqueCurrentFOC(0); // uses Slot 0 by default

    // VoltageOut used only during SysId characterization runs (not normal operation)
    private final VoltageOut m_sysIdVoltageRequest = new VoltageOut(0);

    // ── Distance → RPM interpolation table ───────────────────────────────────
    // Populated at construction from ShooterConstants.RPM_DISTANCE_TABLE.
    // Call setFlywheelRPMFromDistance() to use it.
    private static final InterpolatingDoubleTreeMap m_rpmTable = new InterpolatingDoubleTreeMap();
    static {
        for (double[] entry : ShooterConstants.RPM_DISTANCE_TABLE) {
            m_rpmTable.put(entry[0], entry[1]);
        }
    }

    // ── Fire control solver (shoot-on-the-move) ───────────────────────────────
    // Combines RK4 physics-derived TOF with empirical RPM data.
    // Used by DriveToHubAndShootCommand for Newton-method SOTM correction.
    private final FireControlSolver m_fireControl;

    // ── SysId routine ─────────────────────────────────────────────────────────
    // Initialized in the constructor (after m_flywheelLeader is assigned).
    // Use sysIdQuasistatic() and sysIdDynamic() command factories in RobotContainer.
    // After running: divide reported kS/kV/kA (Volts) by motor Kt (~0.0181 V/A for
    // Kraken X60 / Falcon 500) to get the Amp-domain gains for VelocityTorqueCurrentFOC.
    private final SysIdRoutine m_sysIdRoutine;

    // ── Internal state ────────────────────────────────────────────────────────
    private double  m_targetPivotDeg = 0.0;
    private double  m_targetRPM      = ShooterConstants.IDLE_RPM;
    private boolean m_isShooting     = false;

    // ── Constructor ───────────────────────────────────────────────────────────
    public ShooterSubsystem() {
        CANBus pivotBus    = new CANBus(ShooterConstants.PIVOT_CAN_BUS);
        CANBus flywheelBus = new CANBus(ShooterConstants.FLYWHEEL_CAN_BUS);

        m_pivotLeader    = new TalonFX(ShooterConstants.PIVOT_LEADER_ID,    pivotBus);
        m_pivotFollower  = new TalonFX(ShooterConstants.PIVOT_FOLLOWER_ID,  pivotBus);
        m_flywheelLeader = new TalonFX(ShooterConstants.FLYWHEEL_LEADER_ID, flywheelBus);
        m_flywheelFollower = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_ID, flywheelBus);

        configurePivot();
        configureFlywheel();

        m_fireControl = initFireControl();

        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,           // default ramp rate (1 V/s)
                Volts.of(7),    // max voltage during dynamic test
                null            // default timeout
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> m_flywheelLeader.setControl(
                    m_sysIdVoltageRequest.withOutput(volts.in(Volts))),
                (log) -> log.motor("flywheel-leader")
                    .voltage(Volts.of(m_flywheelLeader.getMotorVoltage().getValueAsDouble()))
                    .angularPosition(Rotations.of(m_flywheelLeader.getPosition().getValueAsDouble()))
                    .angularVelocity(RotationsPerSecond.of(m_flywheelLeader.getVelocity().getValueAsDouble())),
                this
            )
        );
    }

    // ── Motor configuration ───────────────────────────────────────────────────
    private void configurePivot() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.PIVOT_CRUISE_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration   = ShooterConstants.PIVOT_ACCELERATION;

        cfg.Slot0.kP = ShooterConstants.PIVOT_kP;
        cfg.Slot0.kI = ShooterConstants.PIVOT_kI;
        cfg.Slot0.kD = ShooterConstants.PIVOT_kD;

        applyWithRetry(m_pivotLeader, cfg, "Pivot Leader");

        m_pivotFollower.setControl(
            new Follower(ShooterConstants.PIVOT_LEADER_ID, MotorAlignmentValue.Aligned));

        m_pivotLeader.setPosition(0.0);
    }

    private void configureFlywheel() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Stator current limit — prevents brown-outs under load
        cfg.CurrentLimits.StatorCurrentLimit       = ShooterConstants.FLYWHEEL_STATOR_LIMIT_AMPS;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // Slot 0 gains for VelocityTorqueCurrentFOC — units are AMPS (not Volts).
        // Populate kS/kV/kA after tuning in Phoenix Tuner X.
        cfg.Slot0.kP = ShooterConstants.FLYWHEEL_kP;
        cfg.Slot0.kI = ShooterConstants.FLYWHEEL_kI;
        cfg.Slot0.kD = ShooterConstants.FLYWHEEL_kD;
        cfg.Slot0.kS = ShooterConstants.FLYWHEEL_kS;
        cfg.Slot0.kV = ShooterConstants.FLYWHEEL_kV;
        cfg.Slot0.kA = ShooterConstants.FLYWHEEL_kA;

        applyWithRetry(m_flywheelLeader, cfg, "Flywheel Leader");

        m_flywheelFollower.setControl(
            new Follower(ShooterConstants.FLYWHEEL_LEADER_ID,
                ShooterConstants.FLYWHEEL_FOLLOWER_OPPOSE
                    ? MotorAlignmentValue.Opposed
                    : MotorAlignmentValue.Aligned));
    }

    // ── Shooter control methods ────────────────────────────────────────────────
    public void setLaunchAngleDeg(double degrees) {
        m_targetPivotDeg = degrees;
        double rotations = (degrees / 360.0) * ShooterConstants.PIVOT_GEAR_RATIO;
        m_pivotLeader.setControl(m_mmRequest.withPosition(rotations));
    }

    public void setFlywheelRPM(double rpm) {
        m_targetRPM = rpm;
        // TalonFX velocity control is in rotations/second
        double rps = rpm / 60.0;
        m_flywheelLeader.setControl(m_velocityRequest.withVelocity(rps));
    }

    /**
     * Look up the target RPM from the distance→RPM interpolation table and
     * command the flywheel to that speed. Call this with the camera-measured
     * distance to the hub each loop while the robot is aiming.
     *
     * @param distanceMeters Horizontal distance from shooter exit to hub (metres).
     */
    public void setFlywheelRPMFromDistance(double distanceMeters) {
        double rpm = m_rpmTable.get(distanceMeters);
        setFlywheelRPM(rpm);
    }

    public void idleFlywheel() {
        m_targetRPM  = 0;
        m_isShooting = false;
        m_flywheelLeader.stopMotor();
        m_flywheelFollower.stopMotor();
    }

    public boolean isAtTargetRPM(double targetRPM) {
        return Math.abs(getCurrentRPM() - targetRPM) < 100.0;
    }

    /**
     * Returns true when the flywheel is spinning (target > idle) and within
     * 50 RPM of the current target. Used to gate the feeder.
     */
    public boolean isReadyToShoot() {
        return m_targetRPM > ShooterConstants.IDLE_RPM && isAtTargetRPM(m_targetRPM);
    }

    public double getCurrentRPM() {
        // getVelocity() returns rotations/second — multiply by 60 for RPM
        return m_flywheelLeader.getVelocity().getValueAsDouble() * 60.0;
    }

    public boolean isAtTargetAngle(double targetDeg) {
        return Math.abs(getCurrentAngleDeg() - targetDeg)
            < ShooterConstants.PIVOT_ANGLE_TOLERANCE_DEG;
    }

    // ── Teleop helpers ────────────────────────────────────────────────────────
    public void moveToDegrees(double degrees) { setLaunchAngleDeg(degrees); }
    public void setPowerLevel(double power)   { m_pivotLeader.set(power); }

    public void stopMotors() {
        m_pivotLeader.stopMotor();
        m_pivotFollower.stopMotor();
        m_flywheelLeader.stopMotor();
        m_flywheelFollower.stopMotor();
    }

    public double getCurrentAngleDeg() {
        return (m_pivotLeader.getPosition().getValueAsDouble()
            / ShooterConstants.PIVOT_GEAR_RATIO) * 360.0;
    }
    public void shoot() {
        setFlywheelRPM(ShooterConstants.FIXED_SHOT_RPM_M);
        m_isShooting = true;
    }
    // ── SysId command factories ────────────────────────────────────────────────
    // Wire these up in RobotContainer to joystick buttons for characterization.
    // Run quasistatic forward, quasistatic reverse, dynamic forward, dynamic reverse
    // in sequence, then open WPILib SysId analyzer to get kS/kV/kA.
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    // ── Periodic ─────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        SmartDashboard.putNumber ("Shooter/Pivot Angle (deg)",   getCurrentAngleDeg());
        SmartDashboard.putNumber ("Shooter/Pivot Target (deg)",  m_targetPivotDeg);
        SmartDashboard.putBoolean("Shooter/Pivot At Target",     isAtTargetAngle(m_targetPivotDeg));
        SmartDashboard.putNumber ("Shooter/Flywheel RPM",        getCurrentRPM());
        SmartDashboard.putNumber ("Shooter/Flywheel Target RPM", m_targetRPM);
        SmartDashboard.putBoolean("Shooter/At Target RPM",       isAtTargetRPM(m_targetRPM));
        SmartDashboard.putBoolean("Shooter/Is Shooting",         m_isShooting);
        SmartDashboard.putBoolean("Shooter/Ready To Shoot",      isReadyToShoot());
    }

    // ── Fire control ─────────────────────────────────────────────────────────

    /**
     * Returns the FireControlSolver for use in DriveToHubAndShootCommand.
     * Call {@link FireControlSolver#calculate} once per loop cycle.
     */
    public FireControlSolver getFireControlSolver() {
        return m_fireControl;
    }

    /**
     * Build and populate the FireControlSolver at startup.
     * Uses RK4 physics (FireControlSimulator) to derive time-of-flight values,
     * then overrides RPM with the empirically-confirmed RPM_DISTANCE_TABLE.
     * Takes ~200ms — call from constructor, not from periodic().
     */
    private static FireControlSolver initFireControl() {
        // ── Solver config ────────────────────────────────────────────────────
        FireControlSolver.Config cfg = new FireControlSolver.Config();
        cfg.launcherOffsetX      = ShooterConstants.SHOOTER_X_OFFSET_METERS; // 0.20 m
        cfg.launcherOffsetY      = 0.0;
        cfg.minScoringDistance   = 0.5;
        cfg.maxScoringDistance   = 5.5;
        cfg.maxTiltDeg           = 90.0; // disable tilt gate (pitch/roll not wired)
        FireControlSolver solver = new FireControlSolver(cfg);

        // ── Physics simulation to obtain TOF values ──────────────────────────
        // Ball parameters — update ballMassKg and ballDiameterM if game manual
        // specifies different values for the 2026 game piece.
        FireControlSimulator.SimParameters simParams = new FireControlSimulator.SimParameters(
            0.215,                                               // ballMassKg (~9.5 oz FRC ball)
            0.1501,                                              // ballDiameterM (~5.9 in)
            0.47,                                               // dragCoeff (smooth sphere)
            0.0,                                                // magnusCoeff (disabled)
            1.225,                                              // airDensity kg/m³
            ShooterConstants.SHOOTER_EXIT_HEIGHT_METERS,        // 0.52 m
            ShooterConstants.WHEEL_DIAMETER_INCHES * 0.0254,    // 0.1016 m
            ShooterConstants.HUB_TARGET_HEIGHT_METERS,          // 2.64 m
            ShooterConstants.EFFICIENCY,                        // 0.424 slip factor
            ShooterConstants.FIXED_SHOT_ANGLE_DEG,              // 61.5°
            0.001,   // sim timestep (s)
            1500, 6000, 25, 5.0                                  // RPM range, search iters, max time
        );

        FireControlSimulator sim = new FireControlSimulator(simParams);
        FireControlSimulator.GeneratedLUT lut = sim.generateLUT();

        // Load physics-derived entries, then override RPM with empirical table
        for (FireControlSimulator.LUTEntry e : lut.entries()) {
            if (!e.reachable()) continue;
            // Physics RPM from simulation; empirical lookup overrides if in range
            double rpm = m_rpmTable.get(e.distanceM()); // InterpolatingDoubleTreeMap interpolates
            solver.loadLUTEntry(e.distanceM(), rpm, e.tof());
        }

        System.out.printf("[ShooterSubsystem] FireControlSolver ready: %d/%d entries, max %.2f m, LUT took %dms%n",
            lut.reachableCount(), lut.reachableCount() + lut.unreachableCount(),
            lut.maxRangeM(), lut.generationTimeMs());

        return solver;
    }

    // ── Utility ───────────────────────────────────────────────────────────────
    private static void applyWithRetry(TalonFX motor, TalonFXConfiguration cfg, String label) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(cfg);
            if (status.isOK()) return;
        }
        System.out.println("[ShooterSubsystem] " + label + " config failed: " + status);
    }
}
