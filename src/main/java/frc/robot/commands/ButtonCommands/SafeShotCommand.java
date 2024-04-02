package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.commands.Procedures.AdjustFeederNote;
import frc.robot.commands.Procedures.CheckIntakeGotoOut;
import frc.robot.commands.Procedures.SetIntakeUp;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SafeShotCommand extends SequentialCommandGroup{
    public SafeShotCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem){
        addCommands(
            new CheckIntakeGotoOut(armSubsystem, intakeSubsystem,Intake.SHOOTING_POSE),
            new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE),
            new ParallelCommandGroup(
                new SetArmTarget(armSubsystem, Arm.ARM_INTAKE_UNFOLDING_POSE),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> armSubsystem.getArmAngleRads() < Arm.ARM_INTAKE_UNFOLDING_POSE),
                    new SetIntakeTarget(intakeSubsystem, Intake.FOLDED_POSE_INTERNAL)
                )
            ),
            new AdjustFeederNote(feederSubsystem, shooterSubsystem),
            new ParallelCommandGroup(
                new SetShooterSpeed(shooterSubsystem, Shooter.SHOOTER_SPEED),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> armSubsystem.getArmAngleRads() < Arm.ARM_INTAKE_UNFOLDING_POSE + Arm.AT_TARGET_TOLLERANCE),
                    new SetIntakeTarget(intakeSubsystem, Intake.FOLDED_POSE_INTERNAL),
                    new SetArmTarget(armSubsystem, Arm.ARM_SAFE_ZONE_SHOT)
                )
            ),
            new WaitUntilCommand(() -> !feederSubsystem.isHoldingNote())
        );
    }
}
