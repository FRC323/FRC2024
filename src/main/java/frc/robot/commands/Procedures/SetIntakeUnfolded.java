package frc.robot.commands.Procedures;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            new SetArmTarget(armSubsystem, Arm.ARM_INTAKE_UNFOLDING_POSE),
            new SetIntakeTarget(intakeSubsystem, Intake.UNFOLDED_POSE),
            new SetArmTarget(armSubsystem, Arm.ARM_HANDOFF_POSE)
        );
    }
}