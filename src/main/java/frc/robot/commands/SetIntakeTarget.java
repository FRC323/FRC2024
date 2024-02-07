package frc.robot.commands;

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
    public boolean isFinished() {
        return intake.wristIsAtTarget();
    }
}

