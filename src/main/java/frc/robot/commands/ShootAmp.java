package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootAmp extends SequentialCommandGroup{
    public ShootAmp(ArmSubsystem armSubsystem){
        addCommands(
            new SetShooterSpeed(armSubsystem, Constants.Arm.Shooter.SHOOTER_SPEED),
            new WaitUntilCommand(armSubsystem::atShootSpeed),
            new SetFeederSpeed(armSubsystem, Constants.Arm.FEED_SHOOT_SPEED),
            new WaitUntilCommand(()->!armSubsystem.isHoldingNote()),
            // new WaitUntilCommand(1.0),
            new ParallelCommandGroup(
                new SetFeederSpeed(armSubsystem, 0.0),
                new SetShooterSpeed(armSubsystem, 0.0)
            )
        );
    }
}