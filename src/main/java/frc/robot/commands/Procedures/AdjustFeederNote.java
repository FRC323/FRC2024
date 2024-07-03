package frc.robot.commands.Procedures;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Feeder;
import frc.robot.commands.SetCommands.SetFeederPosition;
import frc.robot.subsystems.FeederSubsystem;

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