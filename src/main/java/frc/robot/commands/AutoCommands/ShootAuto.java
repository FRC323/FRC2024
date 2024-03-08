package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.commands.AlignArmForShot;
import frc.robot.commands.AlignWhileDriving;
import frc.robot.commands.SetFeederSpeed;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootAuto extends SequentialCommandGroup{
    public ShootAuto(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, VisionSubsystem visionSubsystem){
        addCommands(
            new SetShooterSpeed(armSubsystem, Constants.Arm.Shooter.SHOOTER_SPEED),
            new ParallelRaceGroup(
                new AlignWhileDriving(driveSubsystem, visionSubsystem, ()->0.0, ()-> 0.0, () ->0.0),
                new AlignArmForShot(armSubsystem, intakeSubsystem, visionSubsystem),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(
                        () -> armSubsystem.armIsAtTarget() && armSubsystem.atShootSpeed(Constants.Arm.Shooter.SHOOTER_SPEED)
                    ),
                    new InstantCommand(()-> armSubsystem.setFeederSpeed(Arm.FEED_SHOOT_SPEED)),
                    new WaitUntilCommand(()->!armSubsystem.isHoldingNote()),
                    new InstantCommand(()-> armSubsystem.setFeederSpeed(0))
                )
            )
        );
    }
}
