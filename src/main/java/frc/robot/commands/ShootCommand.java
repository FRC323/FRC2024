package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends SequentialCommandGroup{
    public ShootCommand(ArmSubsystem armSubsystem, double shooterSpeed){
        addCommands(
            new SetShooterSpeed(armSubsystem, -1.0),
            new WaitCommand(1.0),
            new SetFeederSpeed(armSubsystem, Constants.Arm.FEED_SHOOT_SPEED)
        );

    }
}
