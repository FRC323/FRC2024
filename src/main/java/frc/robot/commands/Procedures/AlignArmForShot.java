package frc.robot.commands.Procedures;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.commands.SetCommands.SetArmNoBlock;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;

public class AlignArmForShot extends SequentialCommandGroup{
    public AlignArmForShot(
        ArmSubsystem armSubsystem,
        ShooterSubsystem shooterSubsystem,
        FeederSubsystem feederSubsystem,
        IntakeSubsystem intakeSubsystem,
        PoseEstimatorSubsystem poseEstimatorSubsystem
    ){
        addCommands(
            new CheckIntakeGotoOut(armSubsystem, intakeSubsystem, Intake.UNFOLDED_POSE),
            new ParallelCommandGroup(
                new SetIntakeTarget(intakeSubsystem, Intake.UNFOLDED_POSE), 
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> armSubsystem.getArmAngleRads() < Arm.ARM_HANDOFF_POSE),
                    new AdjustFeederNote(feederSubsystem,shooterSubsystem),
                    new SetShooterSpeed(shooterSubsystem, Shooter.SHOOTER_SPEED)
                ),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> intakeSubsystem.getWristAngleRads() > Intake.SHOOTING_POSE),
                    new RepeatCommand(
                        new SetArmNoBlock(armSubsystem, poseEstimatorSubsystem::get_armAngle)
                    )
                )
            )
        );
    }
}
