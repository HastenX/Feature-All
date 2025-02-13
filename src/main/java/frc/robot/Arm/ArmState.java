package frc.robot.Arm;

import frc.robot.subsystems.PivotSubsystem.PivotPosition;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpeed;
import frc.robot.subsystems.WristSubsystem.WristPosition;;

public enum ArmState {
    STORED(PivotPosition.STORED, ElevatorPosition.STORED, WristPosition.STORED, IntakeSpeed.STOP), //TODO: DECIDE IF GOOD OR CHANGE LATER

    GROUND_INTAKE(PivotPosition.GROUND_INTAKE, ElevatorPosition.GROUND_INTAKE, WristPosition.GROUND_INTAKE, IntakeSpeed.CORAL_INTAKE),
    SOURCE_INTAKE(PivotPosition.SOURCE_INTAKE, ElevatorPosition.SOURCE_INTAKE, WristPosition.SOURCE_INTAKE, IntakeSpeed.CORAL_INTAKE),
    ALGAE_INTAKE(PivotPosition.ALGAE_INTAKE, ElevatorPosition.ALGAE_INTAKE, WristPosition.ALGAE_INTAKE, IntakeSpeed.ALGAE_INTAKE),

    L_ONE(PivotPosition.L_ONE, ElevatorPosition.L_ONE, WristPosition.SCORING_CORAL, IntakeSpeed.CORAL_INTAKE),
    L_TWO(PivotPosition.L_TWO, ElevatorPosition.L_TWO, WristPosition.SCORING_CORAL, IntakeSpeed.CORAL_INTAKE),
    L_THREE(PivotPosition.L_THREE, ElevatorPosition.L_THREE, WristPosition.SCORING_CORAL, IntakeSpeed.CORAL_INTAKE),
    L_FOUR(PivotPosition.L_FOUR, ElevatorPosition.L_FOUR, WristPosition.SCORING_CORAL, IntakeSpeed.CORAL_INTAKE),

    COBRA_STANCE(PivotPosition.COBRA_STANCE, ElevatorPosition.COBRA_STANCE, WristPosition.STORED, IntakeSpeed.STOP);

    public final PivotPosition pivotPosition;
    public final ElevatorPosition elevatorPosition;
    public final WristPosition wristPosition;
    public final IntakeSpeed intakeSpeed;

    private ArmState(
        PivotPosition pivotPosition, ElevatorPosition elevatorPosition, WristPosition wristPosition, IntakeSpeed intakeSpeed) {
        this.pivotPosition = pivotPosition;
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.intakeSpeed = intakeSpeed;
    }
}
