package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimbCommand extends SequentialCommandGroup{
    public ClimbCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
            new SetIntakeTarget(intakeSubsystem, Intake.UNFOLDED_POSE),
            new SetArmTarget(armSubsystem, Arm.ARM_CLIMB_POSE)
        );
    }
}
