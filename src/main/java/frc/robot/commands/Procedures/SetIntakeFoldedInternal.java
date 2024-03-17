package frc.robot.commands.Procedures;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetIntakeSpeed;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeFoldedInternal extends SequentialCommandGroup{
    public SetIntakeFoldedInternal(IntakeSubsystem intakeSubsystem,ArmSubsystem armSubsystem, FeederSubsystem feederSubsystem){
        addCommands(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE),
                    new SetArmTarget(armSubsystem, Arm.ARM_INTAKE_UNFOLDING_POSE)
                ),
                new InstantCommand(),
                () -> intakeSubsystem.getWristAngleRads() > Intake.FOLDED_POSE_INTERNAL + Constants.MARGIN_OF_ERROR_RADS
            ),
            new SetIntakeTarget(intakeSubsystem, Intake.FOLDED_POSE_INTERNAL),
            new SetArmTarget(armSubsystem, Arm.ARM_DOWN_POSE)
        );
    }
}