package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.TiltConstants;

public class TiltWristSubsystem implements Subsystem {
    
    private SparkMax motor;
    private SparkClosedLoopController feedbackController;
    private SparkAbsoluteEncoder targetEncoder;
    private TiltWristPosition currentTargetPosition;

    public TiltWristSubsystem() {
        //MOTOR CONSTRUCTOR
        motor = new SparkMax(-1, MotorType.kBrushless);
        // DEFAULT TARGET POSITION
        currentTargetPosition = TiltWristPosition.STOWED;
        // CREATES FEEDBACK CONTROLLER 
        feedbackController = motor.getClosedLoopController();
        // CREATES TARGET ENCODER
        targetEncoder = motor.getAbsoluteEncoder();

        configureMotors();
    }
    
    /**Method, configures motor configuration and applies it to motor */
    private void configureMotors() {
        // CONSTRUCTS MOTORCONFIG
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        // MOTORCONFIG
        motorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        // SETS ENCODER OF MOTORCONFIG
        motorConfig.encoder
            .positionConversionFactor(TiltConstants.kConversionFactor);
        // PPIIIIIIIIDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDFF
        motorConfig.closedLoop
            .pidf(TiltConstants.kP, TiltConstants.kI, TiltConstants.kD, TiltConstants.kFF)
            .outputRange(-1, 1)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        // APPLIES MOTORCONFIG TO MOTOR
        motor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Command that sets the tilt and target position of the wrist
     * @param position
     * What position you want to tilt the wrist to
     */
    public Command setTilt(TiltWristPosition position) {
        return runOnce(() -> {
            currentTargetPosition = position;
            feedbackController.setReference(position.degrees, SparkBase.ControlType.kPosition);
        });
    }

    /** Command that waits for robot to reach target tilt within tolerance */
    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return isAtSetPoint();
        });
    }

    public boolean isAtSetPoint() {
        return getError() < TiltConstants.kTolerance;
    }

    private double getError() {
        return Math.abs(Math.abs(currentTargetPosition.degrees) - Math.abs(targetEncoder.getPosition()));
    }
    
    /**Stops Motor Manually */
    public Command stopMotorCommand() {
        return runOnce(() -> {
            motor.set(0);
        });
    }

    /** Enum, holds positional data (degrees) */
    public enum TiltWristPosition{
        CORAL_STATION_INTAKE(-1), STOWED(180), GROUND_INTAKE(-1);

        private final double degrees;
        /**Constructor for WristRotPos
         * @param degrees
         * The angle for a positiom
         */
        private TiltWristPosition(double degrees) {
            this.degrees = degrees;
        }
    }
}