package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
    private final VelocityVoltage m_velocityRequest =
        new VelocityVoltage(0).withEnableFOC(true);

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
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast; // coast so wheel spins down naturally

        // Velocity PID + feedforward (kV is the most important — tune first)
        cfg.Slot0.kP = ShooterConstants.FLYWHEEL_kP;
        cfg.Slot0.kI = ShooterConstants.FLYWHEEL_kI;
        cfg.Slot0.kD = ShooterConstants.FLYWHEEL_kD;
        cfg.Slot0.kS = ShooterConstants.FLYWHEEL_kS;
        cfg.Slot0.kV = ShooterConstants.FLYWHEEL_kV;

        applyWithRetry(m_flywheelLeader, cfg, "Flywheel Leader");

        // Follower opposes leader if the motors face each other on the same shaft
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

    public void idleFlywheel() {
    m_targetRPM  = 0;
    m_isShooting = false;
    m_flywheelLeader.stopMotor();
    m_flywheelFollower.stopMotor();
}

    public void shoot() {
    setFlywheelRPM(ShooterConstants.FIXED_SHOT_RPM_M);
    m_isShooting = true;
}

    public boolean isAtTargetRPM(double targetRPM) {
        return Math.abs(getCurrentRPM() - targetRPM) < 50.0;
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