package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.AdjustArmForShot;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.AlignWhileDriving;
import frc.robot.commands.SetFeederSpeed;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootAuto extends SequentialCommandGroup{
    public ShootAuto(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem){
        addCommands(
            new SetShooterSpeed(armSubsystem, Constants.Arm.Shooter.SHOOTER_SPEED),
            new ParallelRaceGroup(
                // new AlignWhileDriving(driveSubsystem, armSubsystem, visionSubsystem, () -> 0.0,() -> 0.0), 
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(
                        () -> armSubsystem.armIsAtTarget() && armSubsystem.atShootSpeed()
                    ),
                    new SetFeederSpeed(armSubsystem, Constants.Arm.FEED_SHOOT_SPEED),
                    new WaitUntilCommand(()->!armSubsystem.isHoldingNote())
                )
            )
        );
    }
}
