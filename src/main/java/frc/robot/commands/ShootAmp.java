package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ShootAmp extends SequentialCommandGroup{
    public ShootAmp(ArmSubsystem arm){
        addCommands(
            new SetArmTarget(arm, Constants.Arm.ARM_AMP_POSE),
            new ShootCommand(arm, Constants.Arm.Shooter.AMP_SPEED)
        );
    }
}
