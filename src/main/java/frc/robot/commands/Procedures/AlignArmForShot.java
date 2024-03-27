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
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;

public class AlignArmForShot extends SequentialCommandGroup{
    public AlignArmForShot(
        ArmSubsystem armSubsystem,
        ShooterSubsystem shooterSubsystem,
        IntakeSubsystem intakeSubsystem,
        PoseEstimatorSubsystem poseEstimatorSubsystem
    ){
        addCommands(
            new CheckIntakeGotoOut(armSubsystem, intakeSubsystem, Intake.SHOOTING_POSE),
            new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE), 
            new ParallelCommandGroup(
                new SetShooterSpeed(shooterSubsystem, Shooter.SHOOTER_SPEED),
                new RepeatCommand(
                    new SetArmTarget(armSubsystem, poseEstimatorSubsystem::get_armAngle)
                )
            )
        );
    }
}
