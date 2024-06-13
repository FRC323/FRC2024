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
import frc.robot.commands.SetCommands.SetFeederPosition;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AdjustFeederNote extends SequentialCommandGroup{
    public AdjustFeederNote(FeederSubsystem feederSubsystem){
        addCommands(
            // new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_ADJUST_SPEED),
            // new WaitUntilCommand(feederSubsystem::isHoldingNote),
            // new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_STOPED_SPEED),
            new SetFeederPosition(feederSubsystem, Feeder.ADJUST_POSITION),
            new SetFeederPosition(feederSubsystem, Feeder.SHOOT_POSITION)
        );
    }
}