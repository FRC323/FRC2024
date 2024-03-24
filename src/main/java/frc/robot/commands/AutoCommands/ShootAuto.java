package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Feeder;
import frc.robot.commands.Procedures.AlignArmForShot;
import frc.robot.commands.Procedures.AlignWhileDriving;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;

public class ShootAuto extends SequentialCommandGroup{
    public ShootAuto(
        DriveSubsystem driveSubsystem,
        ArmSubsystem armSubsystem, 
        IntakeSubsystem intakeSubsystem, 
        ShooterSubsystem shooterSubsystem, 
        FeederSubsystem feederSubsystem, 
        PoseEstimatorSubsystem poseEstimatorSubsystem
        ){ 
            addCommands(
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitUntilCommand(
                            () -> armSubsystem.armIsAtTarget() 
                            && shooterSubsystem.atShootSpeed(Constants.Shooter.SHOOTER_SPEED)
                            && armSubsystem.armTargetValidSpeakerTarget()
                        ),
                        new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_ADJUST_SPEED),
                        new WaitUntilCommand(()-> !feederSubsystem.isHoldingNote()),
                        new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEED_SHOOT_SPEED),
                        new WaitUntilCommand(feederSubsystem::isHoldingNote),
                        new WaitUntilCommand(() -> !feederSubsystem.isHoldingNote())
                    ),
                    new AlignArmForShot(armSubsystem, shooterSubsystem, intakeSubsystem, poseEstimatorSubsystem)
                    // new AlignWhileDriving(driveSubsystem, visionSubsystem, ()->0.0, ()-> 0.0, () ->0.0)
                )
            );
    }
}
