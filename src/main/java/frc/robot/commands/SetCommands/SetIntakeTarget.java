package frc.robot.commands.SetCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeTarget extends Command {
    double target;
    IntakeSubsystem intake;
    public SetIntakeTarget(IntakeSubsystem intake, double radTarget) {
        addRequirements(intake);
        this.intake = intake;
        target = radTarget;
    }

    @Override
    public void execute() {
        intake.setTargetRads(target);
    }

    @Override
    public void end(boolean interupted){
        if(interupted){
            intake.setTargetRads(intake.getWristAngleRads());
        }
    }

    @Override
    public boolean isFinished() {
        return intake.wristIsAtTarget();
    }
}

