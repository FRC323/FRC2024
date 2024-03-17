package frc.robot.commands.StoreOffsetCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class StoreDrivetrainOffsets extends Command{
    private DriveSubsystem driveSubsystem;

    public StoreDrivetrainOffsets(DriveSubsystem driveSubsystem){
        addRequirements(driveSubsystem);
        this.driveSubsystem = driveSubsystem;
        
        
        System.out.println("Working Stored Drivetrain Offsets Constructor");
    }

    @Override
    public void execute(){
        System.out.println("Saving");
        this.driveSubsystem.setWheelOffsets();
        System.out.println("Saved");
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
