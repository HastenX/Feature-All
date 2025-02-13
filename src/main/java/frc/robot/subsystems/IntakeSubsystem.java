package frc.robot.subsystems;



import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem implements Subsystem {
 
    public boolean isRunning;
    private SparkMax motor;
    private IntakeSpeed intakeSpeed;

    public IntakeSubsystem() {
        motor = new SparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);

        configureMotors();
    }

    private void configureMotors() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
                     // TODO: CHANGE LATER
        config.softLimit
            .forwardSoftLimit(ElevatorConstants.kMAX_LIMIT)
            .reverseSoftLimit(ElevatorConstants.kMIN_LIMIT)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    /**Prepares the speed of arm for when trigger is pressed
     * @param intakeSpeed
     * The speed of the intake being prepared for
     */
    public Command setWaitingIntake(IntakeSpeed intakeSpeed) {
        return runOnce(() ->this.intakeSpeed = intakeSpeed);
    }
    /**Used to run intake based on speed defined by the ArmState enum and the IntakeSpeed enum inside of it. Will run or stop Intake.
     * @param toRunIntake
     * Boolean to determine to stop or run intake
     */
    public Command runIntake(boolean toRunIntake) {
        return toRunIntake 
            ? runOnce(() -> {
                motor.set(intakeSpeed.speed);
                this.isRunning  = intakeSpeed.isRunning;})
            : runOnce(() -> {
                motor.set(IntakeSpeed.STOP.speed);
                this.isRunning  = intakeSpeed.isRunning;});
    }

    public boolean isRunning() {
        return this.isRunning;
    }

    public Command stopMotorCommand() {
        return runOnce(() -> motor.set(0));
    }

    public enum IntakeSpeed {
        CORAL_INTAKE(.75), ALGAE_INTAKE(-1),
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