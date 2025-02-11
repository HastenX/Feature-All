package frc.robot.subsystems;



import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem implements Subsystem {
 
    public boolean isRunning;
    private SparkMax motor;

    public IntakeSubsystem() {
        motor = new SparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);

        configureMotors();
    }

    private void configureMotors() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config.inverted(false); // TODO: CHANGE LATER

        motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command setIntakeSpeed(IntakeSpeed intake) {
        return runOnce(() -> {
            motor.set(intake.speed);
            this.isRunning  = intake.isRunning;
        });
    }

    public boolean isRunning() {
        return this.isRunning;
    }

    public enum IntakeSpeed {
        CORAL_INTAKE(.75), ALGAE_INTAKE(0),
        CORAL_REVERSE(0), ALGAE_REVERSE(0),
        STOP(0);
        double speed;
        boolean isRunning;
        IntakeSpeed(double speed) {
            this.speed = speed;
            if(this.speed == 0) {
                this.isRunning = false;
                return;
            }
            this.isRunning = true;
        }
    }
}