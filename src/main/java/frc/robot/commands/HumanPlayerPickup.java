package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HumanPlayerPickup extends SequentialCommandGroup{
    public HumanPlayerPickup(IntakeSubsystem intake,ArmSubsystem armsSubsystem){
        addCommands(
            new SetArmTarget(armsSubsystem, Constants.Arm.ARM_HUMAN_PLAYER_POSE),
            new ParallelCommandGroup(
                new SetIntakeTarget(intake, Constants.Intake.FOLDED_POSE_INTERNAL),
                new SetFeederSpeed(armsSubsystem, Constants.Arm.FEEDER_INTAKE_SPEED)
            ),
            new WaitUntilCommand(armsSubsystem::isHoldingNote),
            new AdjustFeederNote(armsSubsystem)
        );
    }
}
