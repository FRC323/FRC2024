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

public class SetIntakeFoldedInternal extends SequentialCommandGroup{
    public SetIntakeFoldedInternal(IntakeSubsystem intakeSubsystem,ArmSubsystem armSubsystem){
        addCommands(
            new SetIntakeSpeed(intakeSubsystem, 0),
            new SetFeederSpeed(armSubsystem, 0),
            new ConditionalCommand(
                new InstantCommand(),
                new SequentialCommandGroup(
                    new SetIntakeTarget(intakeSubsystem, Intake.UNFOLDED_POSE),
                    new WaitUntilCommand(intakeSubsystem::wristIsAtTarget),
                    new SetArmTarget(armSubsystem, Constants.Arm.ARM_INTAKE_UNFOLDING_POSE),
                    new WaitUntilCommand(()-> armSubsystem.armIsAtTarget()),
                    new SetIntakeTarget(intakeSubsystem,Constants.Intake.FOLDED_POSE_INTERNAL), 
                    new WaitUntilCommand(()->intakeSubsystem.wristIsAtTarget())
                ),
                () -> intakeSubsystem.getWristAngleRads() <= (Constants.Intake.FOLDED_POSE_INTERNAL + 0.2)
            ),
            new SetArmTarget(armSubsystem, Constants.Arm.ARM_DOWN_POSE)
        );
    }
}