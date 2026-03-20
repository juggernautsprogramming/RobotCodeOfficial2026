package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/**
 * TurretSubsystem — single Kraken X44 (TalonFX), MotionMagic position control.
 *
 * Zero convention
 * ───────────────
 *   0° = straight ahead (forward face of robot).
 *   Positive angles = RIGHT (as seen from above).
 *   Negative angles = LEFT  (as seen from above).
 *
 * Finding your hard-stop angles (one-time setup)
 * ───────────────────────────────────────────────
 *   1. Deploy with soft limits DISABLED (set TURRET_FORWARD_LIMIT_DEG = 9999
 *      and TURRET_REVERSE_LIMIT_DEG = -9999 temporarily).
 *   2. Open Elastic → "Turret" tab.
 *   3. Manually drive the turret right until cords start to pull.
 *      Read "Angle (deg)" — that is your forward (right) hard-stop.
 *   4. Manually drive left until cords pull.
 *      Read "Angle (deg)" — that is your reverse (left) hard-stop.
 *   5. Back each value off ~10° for safety and paste into TurretConstants.
 *   6. Re-enable the limits (restore the real values).
 *
 * Flip-to-other-side behavior
 * ───────────────────────────
 *   If a command requests an angle outside the safe range, the turret
 *   immediately commands the OPPOSITE limit instead of stopping.
 *   This lets the drivetrain rotate to compensate while the turret
 *   repositions — preventing cord damage while keeping target tracking alive.
 *
 * Zeroing procedure
 * ─────────────────
 *   Physically point the turret straight ahead, then press the operator
 *   "Zero Turret" button (wired to zeroPosition() in RobotContainer).
 */
public class TurretSubsystem extends SubsystemBase {

    private final TalonFX m_motor;
    private final MotionMagicVoltage m_mmRequest =
        new MotionMagicVoltage(0).withEnableFOC(true);

    private double  m_targetDeg  = 0.0;
    private boolean m_isFlipped  = false;   // true when last command caused a limit-flip

    // ── Constructor ───────────────────────────────────────────────────────────

    public TurretSubsystem() {
        m_motor = new TalonFX(TurretConstants.TURRET_MOTOR_ID,
                              TurretConstants.TURRET_CAN_BUS);
        configure();
        setupElastic();
    }

    // ── Motor configuration ───────────────────────────────────────────────────

    private void configure() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Flip this if positive degrees turns the wrong direction on the real robot:
        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        // SensorToMechanismRatio makes getPosition() return TURRET rotations,
        // so every position/limit value below is in turret-rotations (not motor rotations).
        cfg.Feedback.SensorToMechanismRatio = TurretConstants.TURRET_GEAR_RATIO;

        cfg.CurrentLimits.StatorCurrentLimit       = TurretConstants.TURRET_STATOR_LIMIT_AMPS;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // Cord-safety soft limits — in TURRET rotations (degrees ÷ 360)
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            TurretConstants.TURRET_FORWARD_LIMIT_DEG / 360.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            TurretConstants.TURRET_REVERSE_LIMIT_DEG / 360.0;

        cfg.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.TURRET_CRUISE_VEL_RPS;
        cfg.MotionMagic.MotionMagicAcceleration   = TurretConstants.TURRET_ACCEL_RPS2;

        cfg.Slot0.kP = TurretConstants.TURRET_kP;
        cfg.Slot0.kI = TurretConstants.TURRET_kI;
        cfg.Slot0.kD = TurretConstants.TURRET_kD;
        cfg.Slot0.kS = TurretConstants.TURRET_kS;
        cfg.Slot0.kV = TurretConstants.TURRET_kV;

        applyWithRetry(cfg);

        // Zero the encoder at startup.
        // Remove this line and use the operator "Zero Turret" button instead
        // if the turret is not guaranteed to be pointing forward at power-on.
        m_motor.setPosition(0.0);
    }

    // ── Elastic / Shuffleboard layout ─────────────────────────────────────────

    private void setupElastic() {
        ShuffleboardTab tab = Shuffleboard.getTab("Turret");

        // Primary position readout — the number to watch while finding limits
        tab.addNumber("Angle (deg)",         this::getAngleDeg)
           .withPosition(0, 0).withSize(2, 2);

        // Raw motor rotations (before gear ratio) — useful if angle looks wrong
        tab.addNumber("Raw Motor Rotations", this::getRawMotorRotations)
           .withPosition(2, 0).withSize(2, 1);

        // Target and error
        tab.addNumber("Target (deg)",        () -> m_targetDeg)
           .withPosition(0, 2).withSize(2, 1);
        tab.addNumber("Error (deg)",         () -> m_targetDeg - getAngleDeg())
           .withPosition(2, 1).withSize(2, 1);

        // Limit proximity indicators — watch these while finding your hard-stop angles
        tab.addBoolean("At Forward Limit",   this::isAtForwardLimit)
           .withPosition(0, 3).withSize(2, 1);
        tab.addBoolean("At Reverse Limit",   this::isAtReverseLimit)
           .withPosition(2, 2).withSize(2, 1);

        // Flip indicator — lights up when the turret was sent to the opposite side
        tab.addBoolean("Flipped to Other Side", () -> m_isFlipped)
           .withPosition(0, 4).withSize(2, 1);

        // Configured limit values — for quick reference in Elastic
        tab.addNumber("Fwd Limit (deg)",     () -> TurretConstants.TURRET_FORWARD_LIMIT_DEG)
           .withPosition(2, 3).withSize(2, 1);
        tab.addNumber("Rev Limit (deg)",     () -> TurretConstants.TURRET_REVERSE_LIMIT_DEG)
           .withPosition(2, 4).withSize(2, 1);

        tab.addBoolean("At Target",          this::isAtTarget)
           .withPosition(4, 0).withSize(2, 1);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Commands the turret to the given angle (degrees, 0 = straight ahead,
     * + = right, − = left).
     *
     * If the requested angle is outside the safe range, the turret flips to
     * the OPPOSITE limit instead of stopping, so the drivetrain can rotate
     * while the turret repositions from the other side.
     */
    public void setAngleDeg(double degrees) {
        double command;

        if (degrees > TurretConstants.TURRET_FORWARD_LIMIT_DEG) {
            // Target is past the right cord limit → jump to the left limit
            command     = TurretConstants.TURRET_REVERSE_LIMIT_DEG;
            m_isFlipped = true;
        } else if (degrees < TurretConstants.TURRET_REVERSE_LIMIT_DEG) {
            // Target is past the left cord limit → jump to the right limit
            command     = TurretConstants.TURRET_FORWARD_LIMIT_DEG;
            m_isFlipped = true;
        } else {
            command     = degrees;
            m_isFlipped = false;
        }

        m_targetDeg = command;
        m_motor.setControl(m_mmRequest.withPosition(command / 360.0));
    }

    /**
     * Stamps the current physical position as 0° (straight ahead).
     * Press the bound operator button while the turret is pointed forward.
     */
    public void zeroPosition() {
        m_motor.setPosition(0.0);
        m_targetDeg = 0.0;
        m_isFlipped = false;
    }

    /** Stops the motor and holds via brake mode. */
    public void stop() {
        m_motor.stopMotor();
    }

    /** Current turret angle in degrees (0 = straight ahead). */
    public double getAngleDeg() {
        // getPosition() already returns mechanism (turret) rotations after SensorToMechanismRatio
        return m_motor.getPosition().getValueAsDouble() * 360.0;
    }

    /** Raw motor encoder position in rotations (before gear ratio). Use for calibration. */
    public double getRawMotorRotations() {
        return m_motor.getPosition().getValueAsDouble() * TurretConstants.TURRET_GEAR_RATIO;
    }

    /** True when within tolerance of the target angle. */
    public boolean isAtTarget() {
        return Math.abs(getAngleDeg() - m_targetDeg) < TurretConstants.TURRET_ANGLE_TOLERANCE_DEG;
    }

    /** True when near the right (forward) soft limit. */
    public boolean isAtForwardLimit() {
        return getAngleDeg() >= TurretConstants.TURRET_FORWARD_LIMIT_DEG - TurretConstants.TURRET_ANGLE_TOLERANCE_DEG;
    }

    /** True when near the left (reverse) soft limit. */
    public boolean isAtReverseLimit() {
        return getAngleDeg() <= TurretConstants.TURRET_REVERSE_LIMIT_DEG + TurretConstants.TURRET_ANGLE_TOLERANCE_DEG;
    }

    /**
     * True if the most recent setAngleDeg() call caused a limit-flip.
     * Use this in auto-aim commands to signal the drivetrain to rotate.
     */
    public boolean isFlipped() { return m_isFlipped; }

    // ── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Shuffleboard lambdas update automatically — nothing extra needed here.
        // Add any additional runtime logic (e.g., auto-flip on proximity) below.
    }

    // ── Utility ───────────────────────────────────────────────────────────────

    private void applyWithRetry(TalonFXConfiguration cfg) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = m_motor.getConfigurator().apply(cfg);
            if (status.isOK()) return;
        }
        System.out.println("[TurretSubsystem] Config failed: " + status);
    }
}
