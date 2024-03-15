package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HumanPlayerPickup extends SequentialCommandGroup{
    public HumanPlayerPickup(IntakeSubsystem intake,ArmSubsystem armSubsystem){
        addCommands(
            new SetIntakeUnfolded(intake, armSubsystem),
            new SetArmTarget(armSubsystem, Constants.Arm.ARM_HUMAN_PLAYER_POSE),
            new ParallelCommandGroup(
                new SetIntakeTarget(intake, Constants.Intake.FOLDED_POSE_INTERNAL),
                new SetFeederSpeed(armSubsystem, Constants.Arm.FEEDER_INTAKE_SPEED)
            ),
            new WaitUntilCommand(armSubsystem::isHoldingNote),
            new AdjustFeederNote(armSubsystem),
            new SetIntakeUnfolded(intake, armSubsystem)
        );
    }
}
