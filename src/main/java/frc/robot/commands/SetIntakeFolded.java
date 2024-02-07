package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeFolded extends Command {
    boolean state;
    IntakeSubsystem intake;
    public SetIntakeFolded(IntakeSubsystem intake, boolean state) {
        addRequirements(intake);
        this.intake = intake;
        this.state = state;
    }

    @Override
    public void execute() {
        intake.setFolded(this.state);;
    }

    @Override
    public boolean isFinished() {
        return intake.wristIsAtTarget();
    }
}