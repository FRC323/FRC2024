package frc.robot.commands;

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
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetIntakeSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HandoffProc extends SequentialCommandGroup{
    public HandoffProc(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem){
        addCommands(
            new SetIntakeUnfolded(intakeSubsystem,armSubsystem),
            new SetArmTarget(armSubsystem, Arm.ARM_HANDOFF_POSE),
            new WaitUntilCommand(armSubsystem::armIsAtTarget),
            new WaitUntilCommand(intakeSubsystem::wristIsAtTarget),
            new ParallelCommandGroup(
                new SetFeederSpeed(armSubsystem, Constants.Arm.FEEDER_INTAKE_SPEED), 
                new SetIntakeSpeed(intakeSubsystem, Constants.Intake.INTAKE_SPEED)
            ),
            new WaitUntilCommand(armSubsystem::isHoldingNote),
                        // new ParallelCommandGroup(
            new AdjustFeederNote(armSubsystem),
            //     new SetIntakeSpeed(intakeSubsystem, -Constants.Intake.INTAKE_SPEED)
            // ),
            // new WaitCommand(0.2),
            new ParallelCommandGroup(
                new SetFeederSpeed(armSubsystem, 0),
                new SetIntakeSpeed(intakeSubsystem, 0)
            )
        );

    }

    
}