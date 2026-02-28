package frc.robot.subsystems.Shooter;

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
 * Shooter subsystem — pivot angle control (Motion Magic) + flywheel RPM stubs.
 *
 * Implements DriveToHubAndShootCommand.IShooterSubsystem so the auto-align
 * command can drive this subsystem without being tightly coupled to hardware.
 *
 * ── Adding flywheel motors ──────────────────────────────────────────────────
 * Three places are marked with FLYWHEEL_STUB comments below.
 * Steps:
 *  1. Add FLYWHEEL_LEADER_ID / FLYWHEEL_FOLLOWER_ID to Constants.ShooterConstants.
 *  2. Uncomment the TalonFX declarations in the FLYWHEEL_STUB (1) block.
 *  3. Uncomment the configuration call in FLYWHEEL_STUB (2).
 *  4. Replace the SmartDashboard stub in setFlywheelRPM() with a real motor call.
 *  5. Replace the stub in isAtTargetRPM() with a real encoder read.
 *  6. Activate your indexer/kicker in shoot().
 *  7. Stop flywheel motors in stopMotors().
 * ───────────────────────────────────────────────────────────────────────────
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
        m_pivotLeader   = new TalonFX(ShooterConstants.PIVOT_LEADER_ID,   ShooterConstants.PIVOT_CAN_BUS);
        m_pivotFollower = new TalonFX(ShooterConstants.PIVOT_FOLLOWER_ID, ShooterConstants.PIVOT_CAN_BUS);

        // FLYWHEEL_STUB (1) continued:
        // m_flywheelLeader   = new TalonFX(ShooterConstants.FLYWHEEL_LEADER_ID,   ShooterConstants.PIVOT_CAN_BUS);
        // m_flywheelFollower = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_ID, ShooterConstants.PIVOT_CAN_BUS);

        configurePivot();

        // FLYWHEEL_STUB (2): Uncomment when flywheel motors are wired.
        // configureFlywheel();
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

        // Phoenix 6 Follower: second argument is MotorAlignmentValue, not boolean.
        // AlignedWithMaster = follower spins in the same direction as the leader.
        // OpposedWithMaster = follower spins in the opposite direction.
        m_pivotFollower.setControl(
            new Follower(ShooterConstants.PIVOT_LEADER_ID, MotorAlignmentValue.Aligned));

        m_pivotLeader.setPosition(0.0);
    }

    // FLYWHEEL_STUB (2) method body:
    // private void configureFlywheel() {
    //     TalonFXConfiguration cfg = new TalonFXConfiguration();
    //     cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    //     cfg.Slot0.kP = 0.1;   // tune these
    //     cfg.Slot0.kI = 0.0;
    //     cfg.Slot0.kD = 0.0;
    //     applyWithRetry(m_flywheelLeader, cfg, "Flywheel Leader");
    //     m_flywheelFollower.setControl(
    //         new Follower(ShooterConstants.FLYWHEEL_LEADER_ID, MotorAlignmentValue.AlignedWithMaster));
    // }

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
        // Replace this stub with your motor velocity control when flywheel is wired:
        //   m_flywheelLeader.setControl(new VelocityVoltage(rpm / 60.0).withEnableFOC(true));
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
        // Activate your indexer/kicker motor here:
        //   m_indexer.set(1.0);
        SmartDashboard.putBoolean("Shooter/Firing", true);
    }

    @Override
    public boolean isAtTargetRPM(double targetRPM) {
        // Replace with actual encoder read when flywheel is wired:
        //   double currentRPM = m_flywheelLeader.getVelocity().getValueAsDouble() * 60.0;
        //   return Math.abs(currentRPM - targetRPM) < targetRPM * 0.03;
        //
        // Returning true so the shoot gate does not permanently block during development.
        return true;
    }

    @Override
    public boolean isAtTargetAngle(double targetDeg) {
        return Math.abs(getCurrentAngleDeg() - targetDeg)
            < ShooterConstants.PIVOT_ANGLE_TOLERANCE_DEG;
    }

    @Override
    public SubsystemBase asSubsystem() { return this; }

    // ── Teleop / manual helpers ───────────────────────────────────────────────

    /** Directly move pivot to degrees (equivalent to setLaunchAngleDeg). */
    public void moveToDegrees(double degrees) {
        setLaunchAngleDeg(degrees);
    }

    /** Open-loop power for pivot — use only for manual testing. */
    public void setPowerLevel(double power) {
        m_pivotLeader.set(power);
    }

    /** Hard-stop all motors. */
    public void stopMotors() {
        m_pivotLeader.stopMotor();
        m_pivotFollower.stopMotor();
        // m_flywheelLeader.stopMotor();   // uncomment when flywheel is wired
        // m_flywheelFollower.stopMotor();
    }

    /** Current pivot angle in degrees (from encoder). */
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

    // ── Utilities ─────────────────────────────────────────────────────────────

    private static void applyWithRetry(TalonFX motor, TalonFXConfiguration cfg, String label) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(cfg);
            if (status.isOK()) return;
        }
        System.out.println("[ShooterSubsystem] " + label + " config failed: " + status);
    }
}