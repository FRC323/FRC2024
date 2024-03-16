package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class GotoAmpPose extends SequentialCommandGroup{
    public GotoAmpPose(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem){
        addCommands(
            // new GotoArmIntakeState(armSubsystem, intakeSubsystem, Arm.ARM_AMP_POSE, Intake.SHOOTING_POSE),
            new SetIntakeNeutral(armSubsystem, intakeSubsystem),
            new ParallelCommandGroup(
                new SetArmTarget(armSubsystem, Constants.Arm.ARM_AMP_POSE),
                new SetShooterSpeed(shooterSubsystem, Constants.Shooter.AMP_SPEED)
            ),
            new WaitUntilCommand(() -> !feederSubsystem.isHoldingNote()),
            new WaitCommand(0.15),
            new SetIntakeUp(armSubsystem, intakeSubsystem)
        );
    }
}
