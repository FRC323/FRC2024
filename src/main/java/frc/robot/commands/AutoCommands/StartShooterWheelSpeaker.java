package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class StartShooterWheelSpeaker extends Command{
    private ArmSubsystem armSubsystem;
    public StartShooterWheelSpeaker(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void execute() {
        armSubsystem.setShooterSpeed(0.1);
    }

}
