package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.SetCommands.SetLimelightBlink;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class AdjustFeederNote extends SequentialCommandGroup{
    public AdjustFeederNote(FeederSubsystem feederSubsystem){
        addCommands(
            new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_ADJUST_SPEED),
            new WaitUntilCommand(() -> !feederSubsystem.isHoldingNote()),
            new SetFeederSpeed(feederSubsystem, -Constants.Feeder.FEEDER_ADJUST_SPEED),
            new WaitUntilCommand(feederSubsystem::isHoldingNote),
            new WaitCommand(0.15),
            new SetFeederSpeed(feederSubsystem, 0),
            new SetLimelightBlink()
        );
    }
}