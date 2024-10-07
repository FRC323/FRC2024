package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ResetOdomFromLimelight extends Command{
    private DriveSubsystem driveSubsystem;
    private boolean finished = false;

    public ResetOdomFromLimelight(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    //THIS SHOULDN'T BE NEEDED ANYMORE
    //BUT I'M TOO LAZY TO REMOVE IT FROM
    //PATHPLANNER PATHS

    @Override
    public void execute(){
        //finished = driveSubsystem.updateOdometry();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
