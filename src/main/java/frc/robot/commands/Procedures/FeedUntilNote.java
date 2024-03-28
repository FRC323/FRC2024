package frc.robot.commands.Procedures;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Feeder;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.subsystems.FeederSubsystem;

public class FeedUntilNote extends SequentialCommandGroup{
    public FeedUntilNote(FeederSubsystem feederSubsystem){
        addCommands(
            new SetFeederSpeed(feederSubsystem, Feeder.FEEDER_INTAKE_SPEED),
            new WaitUntilCommand(feederSubsystem::isHoldingNote),
            new SetFeederSpeed(feederSubsystem, 0.0)
        );
    }
}
