package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveToHubAndShootCommand;

/**
 * ShooterSubsystem — pivot angle control (Motion Magic) + flywheel RPM stubs.
 *
 * <h3>Deprecation fix</h3>
 * {@code new TalonFX(id, "CANBusName")} is deprecated in Phoenix 6 2026.
 * Use {@code new TalonFX(id, new CANBus("CANBusName"))} instead.
 */
public class ShooterSubsystem extends SubsystemBase
        implements DriveToHubAndShootCommand.IShooterSubsystem {

    // ── Pivot hardware ────────────────────────────────────────────────────────
    private final TalonFX m_pivotLeader;
    private final TalonFX m_pivotFollower;
    private final MotionMagicVoltage m_mmRequest =
        new MotionMagicVoltage(0).withEnableFOC(true);

    // FLYWHEEL_STUB (1): Uncomment and add CAN IDs to ShooterConstants when ready.
    // private final TalonFX m_flywheelLeader;
    // private final TalonFX m_flywheelFollower;

    // ── Internal state ────────────────────────────────────────────────────────
    private double  m_targetPivotDeg = 0.0;
    private double  m_targetRPM      = ShooterConstants.IDLE_RPM;
    private boolean m_isShooting     = false;

    // ── Constructor ───────────────────────────────────────────────────────────

    public ShooterSubsystem() {
        // FIX: use CANBus object instead of deprecated TalonFX(int, String)
        CANBus pivotBus = new CANBus(ShooterConstants.PIVOT_CAN_BUS);
        m_pivotLeader   = new TalonFX(ShooterConstants.PIVOT_LEADER_ID,   pivotBus);
        m_pivotFollower = new TalonFX(ShooterConstants.PIVOT_FOLLOWER_ID, pivotBus);

        configurePivot();
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

    // ── IShooterSubsystem implementation ──────────────────────────────────────

    @Override
    public void setLaunchAngleDeg(double degrees) {
        m_targetPivotDeg = degrees;
        double rotations = (degrees / 360.0) * ShooterConstants.PIVOT_GEAR_RATIO;
        m_pivotLeader.setControl(m_mmRequest.withPosition(rotations));
    }

    @Override
    public void setFlywheelRPM(double rpm) {
        m_targetRPM = rpm;
        SmartDashboard.putNumber("Shooter/Flywheel Target RPM", rpm);
    }

    @Override
    public void idleFlywheel() {
        setFlywheelRPM(ShooterConstants.IDLE_RPM);
        m_isShooting = false;
    }

    @Override
    public void shoot() {
        m_isShooting = true;
        SmartDashboard.putBoolean("Shooter/Firing", true);
    }

    @Override
    public boolean isAtTargetRPM(double targetRPM) {
        // Replace with real encoder read when flywheel is wired:
        //   double currentRPM = m_flywheelLeader.getVelocity().getValueAsDouble() * 60.0;
        //   return Math.abs(currentRPM - targetRPM) < targetRPM * 0.01;
        return true; // stub — allows development without flywheel wired
    }

    @Override
    public boolean isAtTargetAngle(double targetDeg) {
        return Math.abs(getCurrentAngleDeg() - targetDeg)
            < ShooterConstants.PIVOT_ANGLE_TOLERANCE_DEG;
    }

    @Override
    public SubsystemBase asSubsystem() { return this; }

    // ── Teleop helpers ────────────────────────────────────────────────────────

    public void moveToDegrees(double degrees)   { setLaunchAngleDeg(degrees); }
    public void setPowerLevel(double power)      { m_pivotLeader.set(power); }

    public void stopMotors() {
        m_pivotLeader.stopMotor();
        m_pivotFollower.stopMotor();
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
        SmartDashboard.putNumber ("Shooter/Flywheel Target RPM", m_targetRPM);
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