package frc.robot.commands.SetCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeSpeed extends Command {
    private double intakeVel = 0;
    private IntakeSubsystem intakeSubsystem;
    public SetIntakeSpeed(IntakeSubsystem intakeSubsystem, double vel) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeVel = vel;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeSpeed(intakeVel);
    }

    @Override
    public void end(boolean interupted){
        if(interupted){
            intakeSubsystem.setIntakeSpeed(0);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
