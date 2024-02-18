package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HumanPlayerPickup extends SequentialCommandGroup{
    public HumanPlayerPickup(IntakeSubsystem intake,ArmSubsystem armsSubsystem){
        addCommands(
            new SetArmTarget(armsSubsystem, Constants.Arm.ARM_HUMAN_PLAYER_POSE),
            new SetIntakeTarget(intake, Constants.Intake.FOLDED_POSE),
            new SetFeederSpeed(armsSubsystem, Constants.Arm.FEEDER_INTAKE_SPEED)
        );
    }
}
