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

public class SetIntakeUnfolded extends SequentialCommandGroup{
    public SetIntakeUnfolded(IntakeSubsystem intakeSubsystem,ArmSubsystem armSubsystem){
        addCommands(
            //It is possible to speed this up by running the intake and arm movement in parralell, but that makes things complicated
            new ConditionalCommand(
                new InstantCommand(),
                new SequentialCommandGroup(
                    new SetArmTarget(armSubsystem,Constants.Arm.ARM_INTAKE_UNFOLDING_POSE),
                    new WaitUntilCommand(()-> armSubsystem.getArmAngleRads() <= (Arm.ARM_INTAKE_UNFOLDING_POSE + 0.1)),
                    new SetIntakeTarget(intakeSubsystem,Constants.Intake.UNFOLDED_POSE),
                    new WaitUntilCommand(()-> intakeSubsystem.getWristAngleRads() > (Intake.UNFOLDED_POSE - 0.1)),
                    new SetArmTarget(armSubsystem, Constants.Arm.ARM_HANDOFF_POSE)
                ),
                () -> intakeSubsystem.getWristAngleRads() > (Constants.Intake.UNFOLDED_POSE - 0.5)
            )
        );
    }
}