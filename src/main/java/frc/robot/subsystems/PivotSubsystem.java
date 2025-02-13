package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class PivotSubsystem implements Subsystem {
    private SparkLimitSwitch bottomLimitSwitch;
    private SparkMaxConfig leadConfig;
    private SparkMax followMotor, leadMotor;
    private AbsoluteEncoder targetEncoder;
    private SparkClosedLoopController feedbackController;
    private PivotPosition currentTargetPosition;

    public PivotSubsystem() {
        // MOTORS
        //Right
        leadMotor = new SparkMax(-1, MotorType.kBrushless);
        //Left
        followMotor = new SparkMax(-1, MotorType.kBrushless);
        // ENCODER
        targetEncoder = leadMotor.getAbsoluteEncoder();
        // PID/FEEDBACK CONTROLLER
        feedbackController = leadMotor.getClosedLoopController();
        // SETS TARGET POSITION
        currentTargetPosition = PivotPosition.STORED;
        
        configureMotors();
    }
    /*Cool, motors are being configured her */
    private void configureMotors() {
        // CONFIGURATION CONSTRUCTORS
        SparkMaxConfig followConfig = new SparkMaxConfig();
        SparkMaxConfig leadConfig = new SparkMaxConfig();
        // CONFIGURATIONS
            // RIGHT MOTOR
        leadConfig
            .inverted(true) // TODO: CONFIRM
            .idleMode(IdleMode.kBrake);
        leadConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, -1) 
            .outputRange(-1.0, 1.0);
        leadConfig.absoluteEncoder
            .positionConversionFactor(PivotConstants.kPOSITIONAL_CONVERSION);
        leadConfig.limitSwitch
            .reverseLimitSwitchEnabled(true);
        leadConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(PivotConstants.kMAX_LIMIT)
            .reverseSoftLimit(PivotConstants.kMIN_LIMIT);

        leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            // LEFT MOROR
        followConfig.apply(followConfig);
        followConfig.follow(leadMotor);
        followConfig.inverted(true); // TODO: CONFIRM
        followMotor.configureAsync(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    /** Changes hieght/angle of pivot.
     * @param pivotPosition
     * enum that has height value for target position.
     */
    public Command setHeight(PivotPosition pivotPosition) {
        return runOnce(() -> {
            currentTargetPosition = pivotPosition;
            feedbackController.setReference(pivotPosition.degrees, SparkBase.ControlType.kPosition);
        });
    }

    /**gets the direction of Pivot, used for determining order of command groups.
     * @param newPosition
     * The new position to determine direction
     */
    public boolean isPivotRising(PivotPosition newPosition) {
        return (newPosition.degrees-currentTargetPosition.degrees > 0 );
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return isAtSetpoint();
        });
    }

    public boolean isAtSetpoint() {
        return getError() < PivotConstants.kTOLERANCE;
    }
    
    private double getError() {
        return Math.abs(Math.abs(targetEncoder.getPosition()) - Math.abs(currentTargetPosition.degrees));
    }

      // COMMAND FACTORIES TO ZERO PIVOT

    /**Runs a WaitUntilCommand, waits until pivot reaches bottom */
    public Command waitWhileLowerPivot() {
        return new WaitUntilCommand(() -> {
            leadMotor.set(-0.2);
            return isAtBottom();
        });
    }
    /**Resets encoder to 0 after zeroing */
    public Command resetEncoder() {
        return runOnce(() -> {
            leadConfig.absoluteEncoder.zeroOffset(0); //TODO: Check this. I think this is how this works, but IDK its 10PM
            leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
    }
    /**Ensures that motor is set to 0 after triggering bottomLimitSwitch*/
    public Command stopMotorCommand() {
        return runOnce(() -> {
            leadMotor.set(0);
        });
    }

    /** Stops the motor manually, ignoring all commands. */
    public void stopMotorManual() {
        leadMotor.set(0);
    }

    /**Enables or disables reverseSoftLimmit
     * @param enabled
     * considers wether (based on boolean data) lowerLimit is to be disable or enabled
     */
    public Command toggleLowerLimit(boolean enabled) {
        return runOnce(()-> {
            leadConfig.softLimit.reverseSoftLimitEnabled(enabled);
            leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
    }

    /**Returns if bottomLimitSwitch has been reached, should not be used publicly */
    private boolean isAtBottom() {
        return bottomLimitSwitch.isPressed();
    }
    
    /**Enum, holds position of pivot.
     * @param degrees
     * Height Pivot must reach to get to state.
     */
    public enum PivotPosition{
        ALGAE_INTAKE(-1),GROUND_INTAKE(-1), SOURCE_INTAKE(-1), STORED(-1), COBRA_STANCE(-1),
        L_ONE(-1), L_TWO(-1), L_THREE(-1), L_FOUR(-1);
        double degrees;
        PivotPosition(double degrees) {
            this.degrees = degrees;
        }
    }
}
