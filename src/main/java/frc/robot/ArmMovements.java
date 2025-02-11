package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RotateWristSubsystem;
import frc.robot.subsystems.TiltWristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.LEDController.TargetRGB;
import frc.robot.subsystems.PivotSubsystem.PivotPosition;
import frc.robot.subsystems.RotateWristSubsystem.RotationalWristPosition;
import frc.robot.subsystems.TiltWristSubsystem.TiltWristPosition;

public class ArmMovements {
    public Command directionToBase = null;

    ElevatorSubsystem elevator;
    PivotSubsystem pivot;
    TiltWristSubsystem tilt;
    RotateWristSubsystem rotate;

    LEDController leds;

    public ArmMovements(ElevatorSubsystem elevator, PivotSubsystem pivot, TiltWristSubsystem tilt, RotateWristSubsystem rotate, LEDController leds) {
        this.elevator =elevator;
        this.pivot = pivot;
        this.tilt = tilt;
        this.rotate = rotate;
        this.leds = leds;
    }

    // ABSTRACTED COMMANDS USED WITH MOVEMENTS

    /**Stops Motors if interrupted */
    public Command stopAllMotorsCommand() {
        return elevator.stopMotorCommand()
            .alongWith(pivot.stopMotorCommand())
            .alongWith(rotate.stopMotorCommand())
            .alongWith(tilt.stopMotorCommand());
    }
    /**Abstracted parts that are consistent, regardless of 
     * @param toRun
     * The Command that is being edited with repetative stuff (i.e.: LEDs and motors)
     * @param directionToBase
     * Holds the corresponding command that would return the robot 
     */
    public Command adjustCommand(Command toRun, Command directionToBase) {
        return toRun
            .beforeStarting(() -> leds.setColor(TargetRGB.IN_TRANSITION))
            .finallyDo(() -> {
                leds.setColor(TargetRGB.AT_POSITION);
                applyDirectionToBase(directionToBase);
            })
            .handleInterrupt(() -> {
                leds.setColor(TargetRGB.INTERUPTED); 
                applyDirectionToBase(directionToBase); // TODO: MAKE SURE THIS WILL NOT CAUSE ROBOT DAMAGE!!!
                stopAllMotorsCommand();
            });
    }

    public Command applyDirectionToBase(Command toBaseCommand) {
        return directionToBase = toBaseCommand;
    }
    // TO STORE 

    // GROUND INTAKE

    public Command groundIntakeToStore() {
        return adjustCommand( 
            elevator.setHeight(ElevatorPosition.STORED)
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint())
                .andThen(pivot.setHeight(PivotPosition.STORED))
                .alongWith(pivot.waitUntilAtSetpoint()
                ),
            null
        );
    }

    public Command lOneToStore() {
        return adjustCommand( 
            elevator.setHeight(ElevatorPosition.STORED)
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint())
                .andThen(pivot.setHeight(PivotPosition.STORED))
                .alongWith(pivot.waitUntilAtSetpoint()
                ),
            null
        );
    }

    public Command lTwoToStore() {
        return adjustCommand( 
            elevator.setHeight(ElevatorPosition.STORED)
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint())
                .andThen(pivot.setHeight(PivotPosition.STORED))
                .alongWith(pivot.waitUntilAtSetpoint()
                ),
            null
        );
    }

    public Command lThreeToStore() {
        return adjustCommand( 
            elevator.setHeight(ElevatorPosition.STORED)
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint())
                .andThen(pivot.setHeight(PivotPosition.STORED))
                .alongWith(pivot.waitUntilAtSetpoint()
                ),
            null
        );
    }

    public Command lFourToStore() {
        return adjustCommand( 
            elevator.setHeight(ElevatorPosition.STORED)
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint())
                .andThen(pivot.setHeight(PivotPosition.STORED))
                .alongWith(pivot.waitUntilAtSetpoint()
                ),
            null
        );
    }

    public Command algaeToStore() {
        return adjustCommand( 
            elevator.setHeight(ElevatorPosition.STORED)
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint())
                .andThen(pivot.setHeight(PivotPosition.STORED))
                .alongWith(pivot.waitUntilAtSetpoint()
                ),
            null
        );
    }

    public Command sourceToStore() {
        return adjustCommand( 
            elevator.setHeight(ElevatorPosition.STORED)
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint())
                .andThen(pivot.setHeight(PivotPosition.STORED))
                .alongWith(pivot.waitUntilAtSetpoint()
                ),
            null
        );
    }

    public Command cobraStanceToStore() {
        return adjustCommand( 
            elevator.setHeight(ElevatorPosition.STORED)
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint())
                .andThen(pivot.setHeight(PivotPosition.STORED))
                .alongWith(pivot.waitUntilAtSetpoint()
                ),
            null
        );   
    }

    // FROM STORE TO RESPECTIVE POSITIONS

    public Command groundIntakeFromStore() {
        return adjustCommand( 
            pivot.setHeight(PivotPosition.GROUND_INTAKE)
                .alongWith(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setHeight(ElevatorPosition.GROUND_INTAKE))
                .alongWith(tilt.setTilt(TiltWristPosition.GROUND_INTAKE),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint()
                ),
            groundIntakeToStore()
        );
    }

    public Command lOneFromStore() {
        return adjustCommand( 
            pivot.setHeight(PivotPosition.L_ONE)
                .alongWith(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setHeight(ElevatorPosition.L_ONE))
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.VERICAL),
                    elevator.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint()
                ),
            lOneToStore()
        );
    }

    public Command lTwoFromStore() {
        return adjustCommand( 
            pivot.setHeight(PivotPosition.L_TWO)
                .alongWith(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setHeight(ElevatorPosition.L_TWO))
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.VERICAL),
                    elevator.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint()
                ),
            lTwoToStore()
        );
    }

    public Command lThreeFromStore() {
        return adjustCommand( 
            pivot.setHeight(PivotPosition.L_THREE)
                .alongWith(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setHeight(ElevatorPosition.L_THREE))
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.VERICAL),
                    elevator.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint()
                ),
            lThreeToStore()
        );
    }

    public Command lFourFromStore() {
        return adjustCommand( 
            pivot.setHeight(PivotPosition.L_FOUR)
                .alongWith(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setHeight(ElevatorPosition.L_FOUR))
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.VERICAL),
                    elevator.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint()
                ),
            lFourToStore()
        );
    }

    public Command algaeFromStore() {
        return adjustCommand( 
            elevator.setHeight(ElevatorPosition.ALGAE_INTAKE)
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint())
                .andThen(pivot.setHeight(PivotPosition.ALGAE_INTAKE))
                .alongWith(pivot.waitUntilAtSetpoint()
                ),
            algaeToStore()
        );
    }

    public Command sourcefromStore() {
        return adjustCommand( 
            elevator.setHeight(ElevatorPosition.STORED)
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint())
                .andThen(pivot.setHeight(PivotPosition.STORED))
                .alongWith(pivot.waitUntilAtSetpoint()
                ),
            sourceToStore()
        );
    }

    public Command cobraStanceFromStore() {
        return adjustCommand( 
            elevator.setHeight(ElevatorPosition.STORED)
                .alongWith(tilt.setTilt(TiltWristPosition.STOWED),
                    rotate.setRotation(RotationalWristPosition.HORIZONTAL),
                    elevator.waitUntilAtSetpoint(),
                    tilt.waitUntilAtSetpoint(),
                    rotate.waitUntilAtSetpoint())
                .andThen(pivot.setHeight(PivotPosition.STORED))
                .alongWith(pivot.waitUntilAtSetpoint()
                ),
            cobraStanceToStore()
        );   
    }
}
