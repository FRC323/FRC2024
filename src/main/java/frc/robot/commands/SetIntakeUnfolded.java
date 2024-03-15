package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeUnfolded extends SequentialCommandGroup{
    public SetIntakeUnfolded(IntakeSubsystem intakeSubsystem,ArmSubsystem armSubsystem){
        addCommands(
            new ConditionalCommand(
                new SetIntakeUnfoldedInternal(intakeSubsystem, armSubsystem),
                new ParallelCommandGroup(
                    new SetIntakeTarget(intakeSubsystem,Constants.Intake.UNFOLDED_POSE),
                    new ConditionalCommand(
                        new InstantCommand(),
                        new SequentialCommandGroup(
                            new WaitUntilCommand(()-> intakeSubsystem.getWristAngleRads() >= Constants.Intake.FOLDED_POSE + 0.5),
                            new SetArmTarget(armSubsystem, Constants.Arm.ARM_HANDOFF_POSE)
                        ),
                        () -> armSubsystem.getArmAngleRads() < Arm.ARM_HANDOFF_POSE
                    )
                ),
                () -> intakeSubsystem.getWristAngleRads() < (Constants.Intake.FOLDED_POSE - 0.05)
            ),
            new WaitUntilCommand(()-> intakeSubsystem.wristIsAtTarget() && armSubsystem.armIsAtTarget())
        );
    }
}