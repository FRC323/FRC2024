package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AlignArmForShot extends SequentialCommandGroup{
    public AlignArmForShot(
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem,
        VisionSubsystem visionSubsystem
    ){
        addCommands(
            new SetIntakeUnfolded(intakeSubsystem, armSubsystem), 
            new SetShooterSpeed(armSubsystem, Arm.Shooter.SHOOTER_SPEED),
            new RepeatCommand(
                new ConditionalCommand(
                    new InstantCommand(), 
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new WaitUntilCommand(intakeSubsystem::wristIsAtTarget),
                            new SetArmTarget(armSubsystem,() -> visionSubsystem.get_armAngle())
                        )
                    ),
                    () -> !visionSubsystem.isShotStateValid()
                )
            )
        );
    }
}
