package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeFolded extends SequentialCommandGroup{
    public SetIntakeFolded(IntakeSubsystem intake,ArmSubsystem armSubsystem){
        addCommands(
            new SetArmTarget(armSubsystem,Constants.Arm.ARM_INTAKE_UNFOLDING_POSE),
            new SetIntakeTarget(intake,Constants.Intake.FOLDED_POSE),
            new SetArmTarget(armSubsystem, Constants.Arm.ARM_DOWN_POSE)
        );
    }
}