package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.WristConstants;

public class WristSubsystem implements Subsystem{
    private SparkMax rotateMotor;
    private SparkMax tiltMotor;

    private SparkClosedLoopController rotateFeedbackController;
    private SparkAbsoluteEncoder rotateTargetEncoder;
    private SparkClosedLoopController tiltFeedbackController;
    private SparkAbsoluteEncoder tiltTargetEncoder;

    private WristPosition currentTargetPosition;
    
    /**Constructor for RotateWristSubsystem class */
    public WristSubsystem() {
        // MOTOR CONSTRUCTOR
        rotateMotor = new SparkMax(-1, MotorType.kBrushless);
        tiltMotor = new SparkMax(-1, MotorType.kBrushless);

        // CREATES FEEDBACK CONTROLLER 
        rotateFeedbackController = rotateMotor.getClosedLoopController();
        tiltFeedbackController = tiltMotor.getClosedLoopController();
        // CREATES TARGET ENCODER
        rotateTargetEncoder = rotateMotor.getAbsoluteEncoder();
        tiltTargetEncoder = tiltMotor.getAbsoluteEncoder();

        configureMotors();
    }
    
    /**Method, configures motor configuration and applies it to motor */
    private void configureMotors() {
        // CONSTRUCTS MOTORCONFIG
        SparkMaxConfig motorConfig = new SparkMaxConfig(); // TODO: DECIDE IF MOTORS NEED SEPERATE CLOSEDLOOP CONFIGS
        // MOTORCONFIG
        motorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        // SETS ENCODER OF MOTORCONFIG
        motorConfig.encoder
            .positionConversionFactor(WristConstants.kConversionFactor);
        // PPIIIIIIIIDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDFF
        motorConfig.closedLoop
            .pidf(WristConstants.kP, WristConstants.kI, WristConstants.kD, WristConstants.kFF)
            .outputRange(-1, 1)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        motorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(WristConstants.kMAX_LIMIT)
            .reverseSoftLimit(WristConstants.kMIN_LIMIT);
        // APPLIES MOTORCONFIG TO MOTOR
        rotateMotor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        tiltMotor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  // TODO: VERIFY THIS!!!
    }

    /** Command that sets the rotation and target position of the wrist
     * @param position
     * What rotation you want to move the wrist to
     */
    public Command setPosition(WristPosition position) {
        return runOnce(() -> {
            currentTargetPosition = position;
            rotateFeedbackController.setReference(position.rotateDegrees, SparkBase.ControlType.kPosition);
            tiltFeedbackController.setReference(position.tiltDegrees, SparkBase.ControlType.kPosition);
        });
    }

    /** Command that waits until rotation set point is reached within the tolerance*/
    public Command rotateWaitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return rotateIsAtSetpoint();
        });
    }

    public Command tiltWaitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return tiltIsAtSetpoint();
        });
    }

    public boolean rotateIsAtSetpoint() {
        return getRotateError() < WristConstants.kTolerance;
    }

    public boolean tiltIsAtSetpoint() {
        return getTiltError() < WristConstants.kTolerance;
    }

    private double getRotateError() {
        return Math.abs(Math.abs(currentTargetPosition.rotateDegrees) - Math.abs(rotateTargetEncoder.getPosition()));
    }

    private double getTiltError() {
        return Math.abs(Math.abs(currentTargetPosition.tiltDegrees) - Math.abs(tiltTargetEncoder.getPosition()));
    }

    /**Stops motors with a command */
    public Command stopMotorCommand() {
        return runOnce(() -> {
            rotateMotor.set(0);
            tiltMotor.set(0);
        });
    }


    /**Enum, holds positional data (degrees) */
    public enum WristPosition{
        STORED(-1,-1), GROUND_INTAKE(-1,-1), SOURCE_INTAKE(-1,-1), ALGAE_INTAKE(-1,-1),
        SCORING_CORAL(-1,-1);

        private final double rotateDegrees;
        private final double tiltDegrees;

        /**Constructor for WristRotPos
         * @param degrees
         * The angle for a positiom
         */
        private WristPosition(double rotateDegrees, double tiltDegrees) {
            this.rotateDegrees = rotateDegrees;
            this.tiltDegrees = tiltDegrees;
        }
    }
}
