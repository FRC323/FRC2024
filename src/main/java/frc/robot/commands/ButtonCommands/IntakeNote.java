package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.Procedures.AdjustFeederNote;
import frc.robot.commands.Procedures.CheckIntakeGotoOut;
import frc.robot.commands.Procedures.FeedUntilNote;
import frc.robot.commands.Procedures.CheckIntakeGotoOut;
import frc.robot.commands.Procedures.SetIntakeUp;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetIntakeSpeed;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNote extends SequentialCommandGroup{
    public IntakeNote(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem,FeederSubsystem feederSubsystem){
        addCommands(
            new CheckIntakeGotoOut(armSubsystem, intakeSubsystem,Intake.FOLDED_POSE_INTERNAL),
            new ParallelCommandGroup(
                new SetIntakeTarget(intakeSubsystem, Intake.UNFOLDED_POSE),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> intakeSubsystem.getWristAngleRads() > Intake.SHOOTING_POSE),
                    new SetArmTarget(armSubsystem, Arm.ARM_HANDOFF_POSE)
                )
            ),
            new ParallelCommandGroup(
                new FeedUntilNote(feederSubsystem),
                new SetIntakeSpeed(intakeSubsystem, Intake.INTAKE_SPEED)
            )
            // new SetIntakeSpeed(intakeSubsystem, Intake.OUTTAKE_SPEED),
            // new AdjustFeederNote(feederSubsystem)
        );

    }

    
}