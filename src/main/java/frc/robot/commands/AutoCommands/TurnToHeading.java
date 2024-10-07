package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TurnToHeading extends Command{

    private DriveSubsystem driveSubsystem;

    public TurnToHeading(
        DriveSubsystem driveSubsystem
    ){
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute(){
        driveSubsystem.driveWithHeading(
            0.0,
            0.0,
            driveSubsystem.getShotState().get_heading(),
            true
        );
    }

    @Override
    public boolean isFinished(){
        return driveSubsystem.atHeading();
    }
}
