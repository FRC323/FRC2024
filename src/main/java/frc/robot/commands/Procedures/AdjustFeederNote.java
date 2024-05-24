package frc.robot.commands.Procedures;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Feeder;
import frc.robot.Constants.Shooter;
import frc.robot.commands.SetCommands.SetLimelightBlink;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.adjustNote; //TODO Not sure why this doesn't work - eli
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AdjustFeederNote extends SequentialCommandGroup{
    public AdjustFeederNote(FeederSubsystem feederSubsystem,ShooterSubsystem shooterSubsystem){
        addCommands(

            //This should be like one command. TODO Probally needs a timeout

            new adjustNote()

            // TODO: Test this

        /*
            new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_ADJUST_SPEED),
            new WaitUntilCommand(feederSubsystem::isHoldingNote),
            // new SetShooterSpeed(shooterSubsystem, Shooter.REVERSE_SPEED),
            new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_REVERSE_ADJUST),
            new WaitUntilCommand(() -> !feederSubsystem.isHoldingNote()),
            // new SetShooterSpeed(shooterSubsystem, 0.0),
            new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_ADJUST_SPEED),
            new WaitUntilCommand(feederSubsystem::isHoldingNote),
            new SetFeederSpeed(feederSubsystem, Feeder.FEEDER_STOPED_SPEED)
        */

        );
    }
}