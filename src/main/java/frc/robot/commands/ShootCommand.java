package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends SequentialCommandGroup{
    public ShootCommand(ArmSubsystem armSubsystem,IntakeSubsystem intakeSubsystem,  double shooterSpeed){
        addCommands(
            // new SetIntakeFoldedInternal(intakeSubsystem, armSubsystem),
            new SetIntakeUnfolded(intakeSubsystem, armSubsystem),
            new SetShooterSpeed(armSubsystem, Constants.Arm.Shooter.SHOOTER_SPEED),
            new WaitUntilCommand(armSubsystem::atShootSpeed),
            new SetFeederSpeed(armSubsystem, Constants.Arm.FEED_SHOOT_SPEED),
            new WaitUntilCommand(()->!armSubsystem.isHoldingNote()),
            new ParallelCommandGroup(
                new SetFeederSpeed(armSubsystem, 0.0),
                new SetShooterSpeed(armSubsystem, 0.0)
            )
        );

    }
}
