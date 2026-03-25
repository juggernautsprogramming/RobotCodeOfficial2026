package frc.robot.subsystems.Feeder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * FeederSubsystem — duty-cycle roller for game-piece feeding.
 *
 * <p>Uses {@link DutyCycleOut} instead of VoltageOut so the motor runs at a
 * fixed fraction of available battery voltage rather than actively compensating
 * for sag.  This reduces battery draw and avoids current spikes when the pack
 * is low.  A 15 A supply-current limit caps worst-case battery load.
 */
public class FeederSubsystem extends SubsystemBase {

    private static final int    MOTOR_ID           = 25;
    private static final String CAN_BUS            = "rio";
    /** Maximum supply current drawn from the battery (Amps). */
    private static final double SUPPLY_CURRENT_LIMIT = 15.0;

    private final TalonFX    feederMotor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public FeederSubsystem() {
        feederMotor = new TalonFX(MOTOR_ID, new CANBus(CAN_BUS));

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Cap battery draw — feeder doesn't need more than this
        config.CurrentLimits.SupplyCurrentLimit       = SUPPLY_CURRENT_LIMIT;
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

    @Override
    public void periodic() {}
}
