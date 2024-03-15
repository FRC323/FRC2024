package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GotoAmpPose extends SequentialCommandGroup{
    public GotoAmpPose(ArmSubsystem armSubsystem,IntakeSubsystem intakeSubsystem){
        addCommands(
            new SetIntakeUnfolded(intakeSubsystem, armSubsystem),
            new RepeatCommand(new ParallelCommandGroup(
                new SetArmTarget(armSubsystem, Arm.ARM_AMP_POSE),
                new SetShooterSpeed(armSubsystem, Arm.Shooter.AMP_SPEED)
            ))
        );
    }
}
