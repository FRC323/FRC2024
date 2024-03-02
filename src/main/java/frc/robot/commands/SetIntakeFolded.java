package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeFolded extends SequentialCommandGroup{
    public SetIntakeFolded(IntakeSubsystem intakeSubsystem,ArmSubsystem armSubsystem){
        addCommands(
            new SetIntakeSpeed(intakeSubsystem, 0),
            new SetFeederSpeed(armSubsystem, 0),
            new ConditionalCommand(
                new SetIntakeUnfolded(intakeSubsystem, armSubsystem),
                new InstantCommand(),
                () -> intakeSubsystem.getWristAngleRads() < Intake.FOLDED_POSE
            ),
            // new ConditionalCommand(
                // new InstantCommand(),
                new ParallelCommandGroup(
                    new SetArmTarget(armSubsystem, Constants.Arm.ARM_DOWN_POSE),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> armSubsystem.getArmAngleRads() >= Arm.ARM_HANDOFF_POSE),
                        new SetIntakeTarget(intakeSubsystem, Constants.Intake.FOLDED_POSE)
                    )
                )
                // () ->intakeSubsystem.getWristAngleRads() <= Intake.FOLDED_POSE - 0.05
            // )
        );
    }
}