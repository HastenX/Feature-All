package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{

    public ElevatorPosition currentTargetPosition;

    protected SparkMax leadMotor, followerMotor;
    protected SparkLimitSwitch bottomLimitSwitch;
    protected SparkMaxConfig followConfig, leadConfig;
    protected AbsoluteEncoder targetEncoder;

    private SparkClosedLoopController feedbackController;



    /** Constructs an elevator. */
    public ElevatorSubsystem() {
        // MOTOR CONTROLLERS
        leadMotor = new SparkMax(-1, MotorType.kBrushless);
        followerMotor = new SparkMax(-1, MotorType.kBrushless);
    
        // ABSOLUTE ENCODER
        targetEncoder = leadMotor.getAbsoluteEncoder();

        // PID CONTROLLER
        feedbackController = leadMotor.getClosedLoopController();
        
        // POSITION
        currentTargetPosition = ElevatorPosition.STORED;

        bottomLimitSwitch = leadMotor.getReverseLimitSwitch();

        configureMotors();
    }

    /** Sets the configurations for each motor. */
    private void configureMotors() {
        // CONFIGURATIONS
        // leading config
        leadConfig = new SparkMaxConfig();
        // following config
        followConfig = new SparkMaxConfig();

            // LEFT MOTOR
        leadConfig
            .inverted(true) // TODO: CONFIRM
            .idleMode(IdleMode.kBrake);
        leadConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, -1) 
            .outputRange(-1.0, 1.0);
        leadConfig.softLimit
            // TODO: Assuming in CM, possibly fix later (One CM heigher than L4, within tolerance)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)

            .forwardSoftLimit(ElevatorConstants.kMAX_LIMIT)
            .reverseSoftLimit(ElevatorConstants.kMIN_LIMIT);
        leadConfig.limitSwitch
            .forwardLimitSwitchEnabled(true) // TODO: Check for changes w/ design (wether or not we will be using limit switches)
            .reverseLimitSwitchEnabled(true)

            .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
            .setSparkMaxDataPortConfig();
        leadConfig.encoder
            .positionConversionFactor(1);
            // APPLIES LEFT CONFIG TO RIGHT MOTOR
        leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            // RIGHT MOTOR
        followConfig.apply(leadConfig);
        followConfig.follow(leadMotor);
        followConfig.inverted(true); // TODO: CONFIRM,
            // APPLIES RIGHT CONFIG TO RIGHT MOTOR
        followerMotor.configureAsync(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    // COMMAND FACTORIES TO REACH ENUM HEIGHT

    /** Sets the target height of the elevator. 
     * @param ElevatorPosition
     * The taregt position: including state and height.
    */
    public Command setHeight(ElevatorPosition position) {
        return runOnce(() -> {
            // CHANGES CURRENT TARGET TO POS
            currentTargetPosition = position;
            // SETS FEEDBACKCONTROLLER TO POS
            feedbackController.setReference(position.centimeters, SparkBase.ControlType.kPosition);
        });
    }

    /**Waits until elevator reaches position within Tolerance.
     * @param ElevatorPosition
     * Enum for elevator height options. 
     */
    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            // TEST FOR IF ELEVATORERROR IS IN TOLERANCE OF TARGETPOSITION
            return isAtSetpoint();
        });
    }
    
    public boolean isAtSetpoint() {
        return (getElevatorError() < ElevatorConstants.kTOLERENCE);
    }

    private double getElevatorError() {
        return Math.abs(Math.abs(targetEncoder.getPosition())- Math.abs(currentTargetPosition.centimeters));
    }
    // // COMMAND FACTORIES TO ZERO ELEVATOR

    /**Runs a WaitUntilCommand, waits until elevator reaches bottom */
    public Command waitWhileLowerElevator() {
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
    
    /** Enum for elevator height options. Contains heightCentimeters, which is the target height in centimeters. */
    public enum ElevatorPosition {
        // ENUMS FOR POSITIONS
            // LEVEL POSITIONS
        L_ONE(45.72), L_TWO(80.01), L_THREE(120.97), L_FOUR(182.88),
            // NON-LEVEL POSTIONS
        ALGAE_INTAKE(-1), GROUND_INTAKE(-1), SOURCE_INTAKE(-1), COBRA_STANCE(-1), STORED(-1);
            // CONSTRUCTOR FOR ENUM'S HEIGHT (CM)
        public final double centimeters;
        /**Constrcutor for height for ElevatorPositions (Enum for Elevator poses)
        * @param centimeters
        * verticle movement in centimeters
        */
        ElevatorPosition (double centimeters) {
            this.centimeters = centimeters;
        }
    }
}
