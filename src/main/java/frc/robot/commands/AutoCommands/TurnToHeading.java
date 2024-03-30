package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;

public class TurnToHeading extends Command{

    private DriveSubsystem driveSubsystem;
    private PoseEstimatorSubsystem poseEstimatorSubsystem;

    public TurnToHeading(
        DriveSubsystem driveSubsystem,
        PoseEstimatorSubsystem poseEstimatorSubsystem
    ){
        this.driveSubsystem = driveSubsystem;
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    }

    @Override
    public void execute(){
        driveSubsystem.driveWithHeading(
            0.0,
            0.0,
            Rotation2d.fromRadians(poseEstimatorSubsystem.get_heading()),
            true
        );
    }

    @Override
    public boolean isFinished(){
        return driveSubsystem.atHeading();
    }
}
