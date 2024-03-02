package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class AdjustFeederNote extends SequentialCommandGroup{
    public AdjustFeederNote(ArmSubsystem armSubsystem){
        addCommands(
            new SetFeederSpeed(armSubsystem, Constants.Arm.FEEDER_ADJUST_SPEED),
            new WaitUntilCommand(() -> !armSubsystem.isHoldingNote()),
            new SetFeederSpeed(armSubsystem, -Constants.Arm.FEEDER_ADJUST_SPEED),
            new WaitUntilCommand(armSubsystem::isHoldingNote),
            new WaitCommand(0.1),
            new SetFeederSpeed(armSubsystem, 0)
        );
    }
}