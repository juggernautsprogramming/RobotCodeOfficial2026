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

    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    private final MotionMagicVoltage motionMagicRequest =
        new MotionMagicVoltage(0).withSlot(0);

    public ClimberSubsystem() {
        CANBus chassisCAN = new CANBus(ClimberConstants.CAN_BUS);
        leaderMotor   = new TalonFX(ClimberConstants.LEADER_ID,   chassisCAN);
        followerMotor = new TalonFX(ClimberConstants.FOLLOWER_ID, chassisCAN);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.StatorCurrentLimit       = ClimberConstants.STATOR_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration   = ClimberConstants.ACCELERATION;

        config.Slot0.kP = ClimberConstants.kP;
        config.Slot0.kI = ClimberConstants.kI;
        config.Slot0.kD = ClimberConstants.kD;
        config.Slot0.kG = ClimberConstants.kG;

        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

        followerMotor.setControl(new Follower(ClimberConstants.LEADER_ID, MotorAlignmentValue.Opposed));
        leaderMotor.setPosition(0);
    }

    public void setPositionDegrees(double degrees) {
        double rotations = (degrees / 360.0) * ClimberConstants.GEAR_RATIO;
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
        double targetRot  = (targetDeg / 360.0) * ClimberConstants.GEAR_RATIO;
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