package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * ClimberSubsystem — dual-motor elevator with gravity compensation.
 *
 * <h3>Deprecation fix</h3>
 * Removed unused {@code import com.ctre.phoenix6.StatusCode}.
 * Replaced {@code new TalonFX(id, "ChassisCAN")} with
 * {@code new TalonFX(id, new CANBus("ChassisCAN"))}.
 */
public class ClimberSubsystem extends SubsystemBase {

    private static final int LEADER_ID   = 15;
    private static final int FOLLOWER_ID = 16;

    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    private static final double kG            = 0.2;
    private static final double GEAR_RATIO    = 1.0;
    private static final double CRUISE_VELOCITY = 100;
    private static final double ACCELERATION    = 200;

    private final MotionMagicVoltage motionMagicRequest =
        new MotionMagicVoltage(0).withSlot(0);

    public ClimberSubsystem() {
        // FIX: use CANBus object instead of deprecated TalonFX(int, String)
        CANBus chassisCAN = new CANBus("ChassisCAN");
        leaderMotor   = new TalonFX(LEADER_ID,   chassisCAN);
        followerMotor = new TalonFX(FOLLOWER_ID, chassisCAN);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration   = ACCELERATION;

        config.Slot0.kP = 2.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Slot0.kG = kG;

        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

        followerMotor.setControl(new Follower(LEADER_ID, MotorAlignmentValue.Opposed));
        leaderMotor.setPosition(0);
    }

    public void setPositionDegrees(double degrees) {
        double rotations = (degrees / 360.0) * GEAR_RATIO;
        leaderMotor.setControl(motionMagicRequest.withPosition(rotations));
    }

    public void setPowerLevel(double power) { leaderMotor.set(power); }

    public void stopMotors() { leaderMotor.stopMotor(); }

    public double getCurrentPosition() {
        return leaderMotor.getPosition().getValueAsDouble();
    }

    /**
     * Returns true when the climber is within tolerance of the given target.
     * @param targetDeg target position in degrees (same units as {@link #setPositionDegrees})
     */
    public boolean isAtPosition(double targetDeg) {
        double targetRot  = (targetDeg / 360.0) * GEAR_RATIO;
        double currentRot = getCurrentPosition();
        return Math.abs(currentRot - targetRot)
            < (ClimberConstants.CLIMB_POSITION_TOLERANCE_DEG / 360.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber ("Climber/Position_rot", getCurrentPosition());
        SmartDashboard.putNumber ("Climber/Position_deg", getCurrentPosition() * 360.0);
    }
}