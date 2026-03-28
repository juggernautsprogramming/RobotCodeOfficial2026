package frc.robot.subsystems.Feeder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

/**
 * FeederSubsystem — duty-cycle roller for game-piece feeding.
 *
 * <p>Uses {@link DutyCycleOut} instead of VoltageOut so the motor runs at a
 * fixed fraction of available battery voltage rather than actively compensating
 * for sag.  This reduces battery draw and avoids current spikes when the pack
 * is low.
 */
public class FeederSubsystem extends SubsystemBase {

    private final TalonFX    feederMotor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public FeederSubsystem() {
        feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID, new CANBus(FeederConstants.FEEDER_CAN_BUS));

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimit       = FeederConstants.FEEDER_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = feederMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Feeder Motor config failed: " + status);
        }

        feederMotor.setPosition(0);
    }

    /**
     * Run the feeder at the given duty cycle.
     *
     * @param dutyCycle fraction of battery voltage, −1.0 to 1.0
     *                  (e.g. 0.55 = 55 % of current battery voltage)
     */
    public void setPower(double dutyCycle) {
        feederMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }

    public void stop() { feederMotor.stopMotor(); }
}
