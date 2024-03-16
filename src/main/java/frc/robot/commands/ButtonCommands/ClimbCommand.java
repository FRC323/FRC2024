package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.GotoArmIntakeState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimbCommand extends SequentialCommandGroup{
    public ClimbCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
            new GotoArmIntakeState(armSubsystem,intakeSubsystem,Constants.Arm.ARM_CLIMB_POSE,Constants.Intake.UNFOLDED_POSE)
        );
    }
}
