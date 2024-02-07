package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeUnfolded extends Command {
    IntakeSubsystem intake;
    ArmSubsystem armSubsystem;

    private CommandState commandState;

    private enum CommandState{
        FOLDED,
        ARM_HEIGHT_REACHED,
        FINISHED
    }

    public SetIntakeUnfolded(IntakeSubsystem intake,ArmSubsystem armSubsystem) {
        addRequirements(intake);
        this.intake = intake;

        addRequirements(armSubsystem);
        this.armSubsystem = armSubsystem;

        this.commandState = CommandState.FOLDED;
    }

    @Override
    public void execute() {
        switch (commandState) {
            case FOLDED:
                armSubsystem.setTargetRads(Constants.Arm.ARM_INTAKE_UNFOLDING_POSE);
                if(armSubsystem.armIsAtTarget()){
                    commandState = CommandState.ARM_HEIGHT_REACHED;
                }
                break;
            case ARM_HEIGHT_REACHED:
                intake.setTargetRads(Constants.Intake.UNFOLDED_POSE);
                if(intake.wristIsAtTarget()){
                    commandState = CommandState.FINISHED;
                }   
        }
    }

    @Override
    public boolean isFinished() {
        return commandState == CommandState.FINISHED;
    }
}