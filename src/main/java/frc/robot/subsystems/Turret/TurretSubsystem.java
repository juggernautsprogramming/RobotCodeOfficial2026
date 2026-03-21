package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/**
 * TurretSubsystem — Kraken X44 (TalonFX), MotionMagic position control.
 *
 * Convention
 * ──────────
 *   0°   = straight ahead.  Positive = RIGHT.  Negative = LEFT.
 *   Hard stops: right +279.75°, left −266.22°.
 *   Soft limits: right +265°, left −251° (15° safety margin each side).
 *
 * Zeroing
 * ───────
 *   Point the turret straight ahead, then press the operator Y button.
 *
 * Limit-flip behaviour
 * ────────────────────
 *   When the turret is driven into a cord-safety limit while the joystick is
 *   still held, it automatically races to the OPPOSITE limit via MotionMagic.
 *   The flip completes as long as the joystick stays held; release to cancel.
 */
public class TurretSubsystem extends SubsystemBase {

    private final TalonFX            m_motor;
    private final MotionMagicVoltage m_mmRequest      = new MotionMagicVoltage(0).withEnableFOC(true);
    private final DutyCycleOut       m_openLoopRequest = new DutyCycleOut(0);

    private double  m_targetDeg  = 0.0;
    private boolean m_isFlipped  = false;
    private boolean m_isFlipping = false;

    // ── Constructor ───────────────────────────────────────────────────────────

    public TurretSubsystem() {
        m_motor = new TalonFX(TurretConstants.TURRET_MOTOR_ID, TurretConstants.TURRET_CAN_BUS);
        configure();
        setupElastic();
    }

    // ── Motor configuration ───────────────────────────────────────────────────

    private void configure() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted    = InvertedValue.Clockwise_Positive; // right = positive

        // SensorToMechanismRatio: getPosition() returns turret rotations, not motor rotations.
        cfg.Feedback.SensorToMechanismRatio = TurretConstants.TURRET_GEAR_RATIO;

        cfg.CurrentLimits.StatorCurrentLimit       = TurretConstants.TURRET_STATOR_LIMIT_AMPS;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // Cord-safety soft limits in turret rotations (degrees ÷ 360).
        // FORWARD must be > REVERSE or Phoenix rejects the config.
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.TURRET_FORWARD_LIMIT_DEG / 360.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.TURRET_REVERSE_LIMIT_DEG / 360.0;

        cfg.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.TURRET_CRUISE_VEL_RPS;
        cfg.MotionMagic.MotionMagicAcceleration   = TurretConstants.TURRET_ACCEL_RPS2;

        cfg.Slot0.kP = TurretConstants.TURRET_kP;
        cfg.Slot0.kI = TurretConstants.TURRET_kI;
        cfg.Slot0.kD = TurretConstants.TURRET_kD;
        cfg.Slot0.kS = TurretConstants.TURRET_kS;
        cfg.Slot0.kV = TurretConstants.TURRET_kV;

        applyWithRetry(cfg);

        m_motor.setPosition(0.0);
    }

    // ── Elastic layout ────────────────────────────────────────────────────────

    private void setupElastic() {
        ShuffleboardTab tab = Shuffleboard.getTab("Turret");

        tab.addNumber ("Angle (deg)",          this::getAngleDeg)       .withPosition(0, 0).withSize(2, 2);
        tab.addNumber ("Target (deg)",         () -> m_targetDeg)        .withPosition(0, 2).withSize(2, 1);
        tab.addNumber ("Error (deg)",          () -> m_targetDeg - getAngleDeg()).withPosition(0, 3).withSize(2, 1);
        tab.addBoolean("At Target",            this::isAtTarget)         .withPosition(2, 0).withSize(2, 1);
        tab.addBoolean("At Forward Limit",     this::isAtForwardLimit)   .withPosition(2, 1).withSize(2, 1);
        tab.addBoolean("At Reverse Limit",     this::isAtReverseLimit)   .withPosition(2, 2).withSize(2, 1);
        tab.addBoolean("Flipped to Other Side",() -> m_isFlipped)        .withPosition(2, 3).withSize(2, 1);
        tab.addNumber ("Motor Output %",       () -> m_motor.getDutyCycle().getValueAsDouble() * 100.0).withPosition(4, 0).withSize(2, 1);
        tab.addNumber ("Stator Current (A)",   () -> m_motor.getStatorCurrent().getValueAsDouble())    .withPosition(4, 1).withSize(2, 1);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Commands the turret to an angle (degrees). 0 = straight ahead,
     * positive = right, negative = left.
     * Requests past a cord-safety limit flip to the opposite limit instead.
     */
    public void setAngleDeg(double degrees) {
        double command;
        if (degrees > TurretConstants.TURRET_FORWARD_LIMIT_DEG) {
            command    = TurretConstants.TURRET_REVERSE_LIMIT_DEG;
            m_isFlipped = true;
        } else if (degrees < TurretConstants.TURRET_REVERSE_LIMIT_DEG) {
            command    = TurretConstants.TURRET_FORWARD_LIMIT_DEG;
            m_isFlipped = true;
        } else {
            command    = degrees;
            m_isFlipped = false;
        }
        m_targetDeg = command;
        m_motor.setControl(m_mmRequest.withPosition(command / 360.0));
    }

    /**
     * Commands the turret to a robot-frame angle with an angular velocity feedforward.
     * Use this for auto-aim where robot rotation must be cancelled in real time.
     *
     * <p>The feedforward is converted to volts via {@code kV} (V·s/rot). With the default
     * {@code TURRET_kV = 0.0} the position loop runs alone; tune kV on the field to make
     * the turret lead during fast robot rotation.
     *
     * @param degrees      Target turret angle, robot-frame. 0 = straight ahead, + = right.
     * @param ffRadPerSec  Feedforward angular velocity (rad/s). Positive = clockwise.
     *                     Pass 0.0 for pure position control.
     */
    public void setAngleDegWithFF(double degrees, double ffRadPerSec) {
        double command;
        if (degrees > TurretConstants.TURRET_FORWARD_LIMIT_DEG) {
            command     = TurretConstants.TURRET_REVERSE_LIMIT_DEG;
            m_isFlipped = true;
        } else if (degrees < TurretConstants.TURRET_REVERSE_LIMIT_DEG) {
            command     = TurretConstants.TURRET_FORWARD_LIMIT_DEG;
            m_isFlipped = true;
        } else {
            command     = degrees;
            m_isFlipped = false;
        }
        m_targetDeg = command;
        // ffRps: desired turret output-shaft velocity in rot/s
        // ffVoltage: additional voltage bias (kV * ffRps). Zero until TURRET_kV is tuned.
        double ffRps     = ffRadPerSec / (2.0 * Math.PI);
        double ffVoltage = ffRps * TurretConstants.TURRET_kV;
        m_motor.setControl(m_mmRequest.withPosition(command / 360.0).withFeedForward(ffVoltage));
    }

    /**
     * Open-loop drive. power [-1, 1]: positive = right, negative = left.
     * Hitting a cord-safety limit while still driving into it triggers a
     * full-speed MotionMagic flip to the opposite limit.
     */
    public void setOpenLoop(double power) {
        double angle = getAngleDeg();

        // Hold the flip until the turret arrives (within 5°) or the stick is released
        if (m_isFlipping) {
            if (Math.abs(angle - m_targetDeg) < 5.0 || Math.abs(power) < 0.05) {
                m_isFlipping = false;
                m_isFlipped  = false;
            } else {
                m_motor.setControl(m_mmRequest.withPosition(m_targetDeg / 360.0));
                return;
            }
        }

        // Trigger a flip when driving into a limit
        if (power > 0 && angle >= TurretConstants.TURRET_FORWARD_LIMIT_DEG - TurretConstants.TURRET_ANGLE_TOLERANCE_DEG) {
            m_isFlipping = true;
            m_isFlipped  = true;
            m_targetDeg  = TurretConstants.TURRET_REVERSE_LIMIT_DEG;
            m_motor.setControl(m_mmRequest.withPosition(m_targetDeg / 360.0));
            return;
        }
        if (power < 0 && angle <= TurretConstants.TURRET_REVERSE_LIMIT_DEG + TurretConstants.TURRET_ANGLE_TOLERANCE_DEG) {
            m_isFlipping = true;
            m_isFlipped  = true;
            m_targetDeg  = TurretConstants.TURRET_FORWARD_LIMIT_DEG;
            m_motor.setControl(m_mmRequest.withPosition(m_targetDeg / 360.0));
            return;
        }

        m_isFlipped = false;
        m_motor.setControl(m_openLoopRequest.withOutput(power));
    }

    /** Stamps the current position as 0° (straight ahead). */
    public void zeroPosition() {
        m_motor.setPosition(0.0);
        m_targetDeg  = 0.0;
        m_isFlipped  = false;
        m_isFlipping = false;
    }

    public void stop() { m_motor.stopMotor(); }

    /** Current turret angle in degrees. 0 = straight ahead. */
    public double getAngleDeg() {
        return m_motor.getPosition().getValueAsDouble() * 360.0;
    }

    public boolean isAtTarget() {
        return Math.abs(getAngleDeg() - m_targetDeg) < TurretConstants.TURRET_ANGLE_TOLERANCE_DEG;
    }

    public boolean isAtForwardLimit() {
        return getAngleDeg() >= TurretConstants.TURRET_FORWARD_LIMIT_DEG - TurretConstants.TURRET_ANGLE_TOLERANCE_DEG;
    }

    public boolean isAtReverseLimit() {
        return getAngleDeg() <= TurretConstants.TURRET_REVERSE_LIMIT_DEG + TurretConstants.TURRET_ANGLE_TOLERANCE_DEG;
    }

    /** True when the last command triggered a limit-flip. */
    public boolean isFlipped() { return m_isFlipped; }

    // ── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {}

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
