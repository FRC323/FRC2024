package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup{
    public ShootCommand(FeederSubsystem feederSubsystem,ShooterSubsystem shooterSubsystem){
        addCommands(
            new WaitUntilCommand(() -> shooterSubsystem.atShootSpeed()),
            new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEED_SHOOT_SPEED),
            new WaitUntilCommand(() -> !feederSubsystem.isHoldingNote())
        );
    }
}
