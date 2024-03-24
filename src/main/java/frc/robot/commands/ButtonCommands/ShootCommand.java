package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup{
    public ShootCommand(FeederSubsystem feederSubsystem,ShooterSubsystem shooterSubsystem){
        addCommands(
            new ParallelRaceGroup(
                new SetShooterSpeed(shooterSubsystem, Shooter.SHOOTER_SPEED),
                new WaitCommand(1.5)
            ),
            new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_ADJUST_SPEED),
            new WaitUntilCommand(()-> !feederSubsystem.isHoldingNote()),
            new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEED_SHOOT_SPEED),
            new WaitUntilCommand(feederSubsystem::isHoldingNote),
            new WaitUntilCommand(() -> !feederSubsystem.isHoldingNote())
        );
    }
}
