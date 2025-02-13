package frc.robot.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.LEDController.TargetRGB;

public class ArmMovements {
    IntakeSubsystem intake;
    ElevatorSubsystem elevator;
    PivotSubsystem pivot;
    WristSubsystem wrist;

    LEDController leds;

    public ArmMovements(
        ElevatorSubsystem elevator, PivotSubsystem pivot, WristSubsystem wrist, IntakeSubsystem intake, LEDController leds) {
        this.elevator =elevator;
        this.pivot = pivot;
        this.wrist = wrist;
        this.intake = intake;
        this.leds = leds;
    }

    /**Stops Motors if interrupted */
    private Command stopAllMotorsCommand() {
        return elevator.stopMotorCommand()
            .alongWith(pivot.stopMotorCommand())
            .alongWith(wrist.stopMotorCommand())
            .alongWith(intake.stopMotorCommand())
            .ignoringDisable(true);
    }
    /**Abstracted parts that are consistent, regardless of 
     * @param toRun
     * The Command that is being edited with repetative stuff (i.e.: LEDs and motors)
     * @param directionToBase
     * Holds the corresponding command that would return the robot 
     */
    private Command adjustCommand(Command toRun) {
        return toRun
            .beforeStarting(() -> {
                leds.setColor(TargetRGB.IN_TRANSITION);
            })
            .finallyDo(() -> {
                leds.setColor(TargetRGB.AT_POSITION);
            })
            .handleInterrupt(() -> {
                leds.setColor(TargetRGB.INTERUPTED); 
                stopAllMotorsCommand();
            });
    }
    /**Command to move the arm based on wether the Pivot is raising or lowering. Uses the adjustCommand to abstract repetativeness
     * @param targetState
     * The super-enum that contains the other enum-positional states
     */
    public Command setArmState(ArmState targetState) {
        return adjustCommand(pivot.isPivotRising(targetState.pivotPosition) 
            // Runs this code if pivot is raising
            ? adjustCommand(pivot.setHeight(targetState.pivotPosition)
                .andThen(wrist.setPosition(targetState.wristPosition))
                .andThen(elevator.setHeight(elevator.currentTargetPosition)))
                .finallyDo(()->intake.setWaitingIntake(targetState.intakeSpeed))
            // Runs this code if pivot is not raising
            : adjustCommand(elevator.setHeight(targetState.elevatorPosition)
                .andThen(wrist.setPosition(targetState.wristPosition))
                .andThen(pivot.setHeight(targetState.pivotPosition)))
                .finallyDo(()->intake.setWaitingIntake(targetState.intakeSpeed))
            );
    }
}
