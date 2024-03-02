package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootSpeaker extends SequentialCommandGroup{
    public ShootSpeaker(ArmSubsystem arm, IntakeSubsystem intakeSubsystem){
        addCommands(
            new SetArmTarget(arm, Constants.Arm.ARM_HANDOFF_POSE),
            new ShootCommand(arm, intakeSubsystem,  0.1)//Constants.Arm.Shooter.SPEAKER_SPEED)
        );
    }
}
