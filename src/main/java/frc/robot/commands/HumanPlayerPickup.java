package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class HumanPlayerPickup extends SequentialCommandGroup{
    public HumanPlayerPickup(ArmSubsystem armsSubsystem){
        addCommands(
            new SetArmTarget(armsSubsystem, Constants.Arm.ARM_HUMAN_PLAYER_POSE),
            new SetFeederSpeed(armsSubsystem, Constants.Arm.FEEDER_INTAKE_SPEED)
        );
    }
}
