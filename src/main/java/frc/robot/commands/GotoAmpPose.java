package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class GotoAmpPose extends ParallelCommandGroup{
    public GotoAmpPose(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem){
        addCommands(
            new GotoArmIntakeState(armSubsystem, intakeSubsystem, Arm.ARM_AMP_POSE, Intake.SHOOTING_POSE),
            new SetShooterSpeed(shooterSubsystem, Constants.Shooter.AMP_SPEED)
        );
    }
}
