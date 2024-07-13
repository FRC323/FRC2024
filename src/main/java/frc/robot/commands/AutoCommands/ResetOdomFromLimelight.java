package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ResetOdomFromLimelight extends Command{
    private DriveSubsystem driveSubsystem;
    private boolean finished = false;

    public ResetOdomFromLimelight(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        finished = driveSubsystem.updateOdometry();
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}
