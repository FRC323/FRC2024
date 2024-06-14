package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.Procedures.AdjustFeederNote;
import frc.robot.commands.Procedures.CheckIntakeGotoOut;
import frc.robot.commands.Procedures.SetIntakeUp;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class GotoAmpPose extends SequentialCommandGroup{
    public GotoAmpPose(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem){
        addCommands(

            //new WaitUntilCommand(() -> feederSubsystem.isHoldingNote()),

            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE),
                    new SetArmTarget(armSubsystem, Constants.Arm.ARM_AMP_POSE),
                    new SetIntakeTarget(intakeSubsystem, Intake.FOLDED_POSE_INTERNAL)
                ),
                new SetArmTarget(armSubsystem, Constants.Arm.ARM_AMP_POSE),
                () -> intakeSubsystem.getWristAngleRads() > Intake.FOLDED_POSE_INTERNAL + Constants.MARGIN_OF_ERROR_RADS
            ),

            new WaitUntilCommand(() -> !(feederSubsystem.isHoldingNote())),

            new SetIntakeUp(armSubsystem, intakeSubsystem)

            // // new GotoArmIntakeState(armSubsystem, intakeSubsystem, Arm.ARM_AMP_POSE, Intake.SHOOTING_POSE),
            // new CheckIntakeGotoOut(armSubsystem, intakeSubsystem,Intake.SHOOTING_POSE),
            // new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE),
            // new ParallelCommandGroup(
            //     new SetArmTarget(armSubsystem, Constants.Arm.ARM_AMP_POSE),
            //     new SequentialCommandGroup(
            //         new WaitUntilCommand(() -> armSubsystem.getArmAngleRads() < Arm.ARM_HANDOFF_POSE),    
            //         new AdjustFeederNote(feederSubsystem)
            //     ),
            //     new SequentialCommandGroup(
            //         new WaitUntilCommand(() -> armSubsystem.getArmAngleRads() < Arm.ARM_INTAKE_UNFOLDING_POSE),
            //         new SetIntakeTarget(intakeSubsystem, Intake.FOLDED_POSE_INTERNAL)
            //     )
            // ),
            // new ParallelCommandGroup(
            //     new SetShooterSpeed(shooterSubsystem, Constants.Shooter.AMP_SPEED),
            //     new SequentialCommandGroup(
            //         new WaitUntilCommand(() -> armSubsystem.getArmAngleRads() < Arm.ARM_INTAKE_UNFOLDING_POSE),
            //         new SetIntakeTarget(intakeSubsystem, Intake.FOLDED_POSE_INTERNAL)
            //     )
            // ),
            // new WaitUntilCommand(() -> !(feederSubsystem.isHoldingNote())),
            // new WaitCommand(0.5),
            // new SetShooterSpeed(shooterSubsystem, 0.0),
            // new SetIntakeUp(armSubsystem, intakeSubsystem)
        );
    }
}
