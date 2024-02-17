package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ShootSpeaker extends SequentialCommandGroup{
    public ShootSpeaker(ArmSubsystem arm){
        addCommands(
            new SetArmTarget(arm, Constants.Arm.ARM_HANDOFF_POSE),
            new ShootCommand(arm, Constants.Arm.Shooter.SPEAKER_SPEED)
        );
    }
}
