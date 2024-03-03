package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AdjustArmForShot;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.SetFeederSpeed;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootAuto extends SequentialCommandGroup{
    public ShootAuto(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem){
        addCommands(
            new SetShooterSpeed(armSubsystem, Constants.Arm.Shooter.SHOOTER_SPEED),
            // new ParallelCommandGroup(
            //     new AlignToTarget(visionSubsystem, driveSubsystem, Constants.AprilTags.APRILTAG_HEIGHT, Constants.AprilTags.Speaker.TAGS_CENTER),
            //     new AdjustRobotForShot(driveSubsystem, armSubsystem, visionSubsystem)
            // ),
            new SetFeederSpeed(armSubsystem, Constants.Arm.FEED_SHOOT_SPEED),
            new WaitCommand(0.25)
        );
    }
}
