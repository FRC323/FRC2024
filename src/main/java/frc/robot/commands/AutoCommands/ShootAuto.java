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
import frc.robot.Constants.Intake;
import frc.robot.commands.Procedures.AdjustFeederNote;
import frc.robot.commands.Procedures.AlignArmForShot;
import frc.robot.commands.Procedures.AlignWhileDriving;
import frc.robot.commands.Procedures.CheckIntakeGotoOut;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import frc.robot.utils.ShotState;

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
                // new CheckIntakeGotoOut(armSubsystem, intakeSubsystem, Intake.SHOOTING_POSE), 
                //Todo: Make sure filtering doesn't break robot
                new ParallelCommandGroup(
                    new SetArmTarget(armSubsystem, poseEstimatorSubsystem::get_armAngle),
                    new TurnToHeading(driveSubsystem, poseEstimatorSubsystem)
                ),
                new AdjustFeederNote(feederSubsystem, shooterSubsystem),
                new SetShooterSpeed(shooterSubsystem, poseEstimatorSubsystem::get_shooterSpeed),
                new WaitUntilCommand(
                    shooterSubsystem::atShootSpeed
                ),
                new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEED_SHOOT_SPEED),
                new WaitUntilCommand(() -> !feederSubsystem.isHoldingNote()),
                new SetFeederSpeed(feederSubsystem, Feeder.FEEDER_STOPED_SPEED),
                new SetShooterSpeed(shooterSubsystem, 0.0)
            );
    }
}
