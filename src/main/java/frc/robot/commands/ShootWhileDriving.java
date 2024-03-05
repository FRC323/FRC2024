package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootWhileDriving extends SequentialCommandGroup{
    public ShootWhileDriving(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem,IntakeSubsystem intakeSubsystem, VisionSubsystem visionSubsystem, CommandJoystick driveStick){
        addCommands(
            // new SetIntakeFoldedInternal(intakeSubsystem, armSubsystem),
            new SetIntakeUnfolded(intakeSubsystem, armSubsystem), 
            new ParallelRaceGroup(
                // new AlignWhileDriving(driveSubsystem, armSubsystem, visionSubsystem, () -> -driveStick.getY(),() -> -driveStick.getX(), rotStick.getX()),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(
                        () -> armSubsystem.armIsAtTarget() && armSubsystem.atShootSpeed()
                    ),
                    new SetFeederSpeed(armSubsystem, Constants.Arm.FEED_SHOOT_SPEED),
                    new WaitUntilCommand(()->!armSubsystem.isHoldingNote())
                )
            ),
            new SetShooterSpeed(armSubsystem, 0),
            new SetFeederSpeed(armSubsystem, 0)
        );

    }
}
