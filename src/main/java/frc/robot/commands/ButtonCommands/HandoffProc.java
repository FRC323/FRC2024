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
import frc.robot.commands.AdjustFeederNote;
import frc.robot.commands.FeedUntilNote;
import frc.robot.commands.SetIntakeUp;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetIntakeSpeed;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HandoffProc extends SequentialCommandGroup{
    public HandoffProc(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem,FeederSubsystem feederSubsystem){
        addCommands(
            new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE),
            new ParallelCommandGroup(
                new SetIntakeTarget(intakeSubsystem, Intake.UNFOLDED_POSE),
                new SetArmTarget(armSubsystem, Arm.ARM_HANDOFF_POSE)
            ),
            new ParallelCommandGroup(
                new FeedUntilNote(feederSubsystem),
                new SetIntakeSpeed(intakeSubsystem, Intake.INTAKE_SPEED)
            ),
            new SetIntakeSpeed(intakeSubsystem, 0),
            new AdjustFeederNote(feederSubsystem),
            new SetIntakeUp(armSubsystem, intakeSubsystem)




            // new SetIntakeTarget(armSubsystem, intakeSubsystem, Constants.Arm.ARM_HANDOFF_POSE, Constants.Intake.UNFOLDED_POSE),
            // new ParallelCommandGroup(
            //     new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_INTAKE_SPEED), 
            //     new SetIntakeSpeed(intakeSubsystem, Constants.Intake.INTAKE_SPEED)
            // ),
            // new WaitUntilCommand(feederSubsystem::isHoldingNote),
            // new AdjustFeederNote(feederSubsystem),
            // new WaitCommand(0.2)
        );

    }

    
}