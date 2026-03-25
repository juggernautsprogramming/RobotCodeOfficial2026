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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/**
 * TurretSubsystem — Kraken X44 (TalonFX), MotionMagic position control.
 *
 * Convention
 * ──────────
 *   0°   = straight ahead.  Positive = RIGHT.  Negative = LEFT.
 *   Hard stops: right +279.75°, left −266.22°.
 *   Soft limits: right +260°, left −260° (safety margin each side).
 *
 * Zeroing (REQUIRED before every match)
 * ──────────────────────────────────────
 *   1. Point the turret straight ahead physically.
 *   2. Press operator Y button to call zeroPosition().
 *   3. Confirm "Zeroed" indicator in Elastic turns green before enabling auto-aim.
 *
 * Limit-flip behaviour
 * ────────────────────
 *   When the turret is driven into a cord-safety limit while the joystick is
 *   still held, it automatically races to the OPPOSITE limit via MotionMagic.
 */
public class TurretSubsystem extends SubsystemBase {

    private final TalonFX            m_motor;
    private final MotionMagicVoltage m_mmRequest      = new MotionMagicVoltage(0).withEnableFOC(true);
    private final DutyCycleOut       m_openLoopRequest = new DutyCycleOut(0);

    private double  m_targetDeg  = 0.0;
    private boolean m_isFlipped  = false;
    private boolean m_isFlipping = false;

    /**
     * True once the operator has pressed Y to zero the turret this session.
     * Odometry aim should not be used until this is true.
     */
    private boolean m_hasBeenZeroed = false;

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

        // SensorToMechanismRatio: getPosition() returns turret output-shaft rotations.
        cfg.Feedback.SensorToMechanismRatio = TurretConstants.TURRET_GEAR_RATIO; // 20.0

        cfg.CurrentLimits.StatorCurrentLimit       = TurretConstants.TURRET_STATOR_LIMIT_AMPS;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // Soft limits in turret output-shaft rotations (degrees ÷ 360).
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

        // Zero encoder at boot. Operator MUST press Y to rezero if turret
        // is not physically pointing straight ahead at power-on.
        m_motor.setPosition(0.0);
    }

    // ── Elastic / SmartDashboard layout ──────────────────────────────────────

    private void setupElastic() {
        ShuffleboardTab tab = Shuffleboard.getTab("Turret");

        tab.addNumber ("Angle (deg)",           this::getAngleDeg)                                        .withPosition(0, 0).withSize(2, 2);
        tab.addNumber ("Target (deg)",          () -> m_targetDeg)                                        .withPosition(0, 2).withSize(2, 1);
        tab.addNumber ("Error (deg)",           () -> m_targetDeg - getAngleDeg())                        .withPosition(0, 3).withSize(2, 1);
        tab.addBoolean("At Target",             this::isAtTarget)                                         .withPosition(2, 0).withSize(2, 1);
        tab.addBoolean("At Forward Limit",      this::isAtForwardLimit)                                   .withPosition(2, 1).withSize(2, 1);
        tab.addBoolean("At Reverse Limit",      this::isAtReverseLimit)                                   .withPosition(2, 2).withSize(2, 1);
        tab.addBoolean("Flipped to Other Side", () -> m_isFlipped)                                        .withPosition(2, 3).withSize(2, 1);
        tab.addBoolean("Zeroed",                () -> m_hasBeenZeroed)                                    .withPosition(4, 0).withSize(2, 1);
        tab.addNumber ("Motor Output %",        () -> m_motor.getDutyCycle().getValueAsDouble() * 100.0)  .withPosition(4, 1).withSize(2, 1);
        tab.addNumber ("Stator Current (A)",    () -> m_motor.getStatorCurrent().getValueAsDouble())      .withPosition(4, 2).withSize(2, 1);
    }

    // ── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Turret/Zeroed",            m_hasBeenZeroed);
        SmartDashboard.putBoolean("Turret/WARNING_NotZeroed", !m_hasBeenZeroed);
        SmartDashboard.putNumber ("Turret/AngleDeg",          getAngleDeg());
        SmartDashboard.putNumber ("Turret/TargetDeg",         m_targetDeg);
        SmartDashboard.putBoolean("Turret/AtTarget",          isAtTarget());
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Commands the turret to an angle (degrees, robot-frame).
     * Requests past a cord-safety limit flip to the opposite limit instead.
     */
    public void setAngleDeg(double degrees) {
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
        m_motor.setControl(m_mmRequest.withPosition(command / 360.0));
    }

    /**
     * Commands the turret to a robot-frame angle with angular velocity feedforward.
     *
     * <p>Pass the robot's raw angular velocity (rad/s, positive = CCW).
     * This method negates it internally so the turret counter-rotates to stay
     * locked on the hub. Do NOT pre-negate the value at the call site.
     *
     * <p>FF is zero until TURRET_kV is tuned on the field.
     *
     * @param degrees     Target angle (robot-frame). 0 = forward, + = right.
     * @param ffRadPerSec Robot angular velocity rad/s, positive = CCW. NOT pre-negated.
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

        // Negate here — ONE place only. Turret must rotate opposite to robot.
        // ffVoltage is zero until TURRET_kV is tuned.
        double ffRps     = -ffRadPerSec / (2.0 * Math.PI);
        double ffVoltage = ffRps * TurretConstants.TURRET_kV;
        m_motor.setControl(m_mmRequest.withPosition(command / 360.0).withFeedForward(ffVoltage));
    }

    /**
     * Open-loop drive. power [-1, 1]: positive = right, negative = left.
     * Driving into a limit triggers a MotionMagic flip to the opposite limit.
     */
    public void setOpenLoop(double power) {
        double angle = getAngleDeg();

        if (m_isFlipping) {
            if (Math.abs(angle - m_targetDeg) < 5.0 || Math.abs(power) < 0.05) {
                m_isFlipping = false;
                m_isFlipped  = false;
            } else {
                m_motor.setControl(m_mmRequest.withPosition(m_targetDeg / 360.0));
                return;
            }
        }

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

    /**
     * Stamps the current physical position as 0° (straight ahead).
     * MUST be called with the turret physically pointing straight forward.
     */
    public void zeroPosition() {
        m_motor.setPosition(0.0);
        m_targetDeg     = 0.0;
        m_isFlipped     = false;
        m_isFlipping    = false;
        m_hasBeenZeroed = true;
    }

    public void stop() {
        m_motor.stopMotor();
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /** Current turret angle in degrees. 0 = straight ahead, + = right, - = left. */
    public double getAngleDeg() {
        return m_motor.getPosition().getValueAsDouble() * 360.0;
    }

    /** True once the operator has zeroed the turret this session. */
    public boolean hasBeenZeroed() {
        return m_hasBeenZeroed;
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

    public boolean isFlipped() {
        return m_isFlipped;
    }

    // ── Utility ───────────────────────────────────────────────────────────────

    private void applyWithRetry(TalonFXConfiguration cfg) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = m_motor.getConfigurator().apply(cfg);
            if (status.isOK()) return;
        }
        System.out.println("[TurretSubsystem] Config failed after 5 attempts: " + status);
    }
}