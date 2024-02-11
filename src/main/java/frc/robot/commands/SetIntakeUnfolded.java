package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeUnfolded extends SequentialCommandGroup{
    public SetIntakeUnfolded(IntakeSubsystem intake,ArmSubsystem armSubsystem){
        addCommands(
            //It is possible to speed this up by running the intake and arm movement in parralell, but that makes things complicated
            new ConditionalCommand(
                new InstantCommand(),
                new SetArmTarget(armSubsystem,Constants.Arm.ARM_INTAKE_UNFOLDING_POSE),
                () -> intake.getWristAngleRads() > Constants.Intake.UNFOLDED_POSE - 0.5
                ),
            //TODO: Check if already unfolded before moving
            new SetIntakeTarget(intake,Constants.Intake.UNFOLDED_POSE),
            new SetArmTarget(armSubsystem, Constants.Arm.ARM_HANDOFF_POSE)
        );
    }
}