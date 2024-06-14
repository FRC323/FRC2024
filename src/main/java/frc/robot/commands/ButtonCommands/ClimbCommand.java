package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.Procedures.CheckIntakeGotoOut;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimbCommand extends SequentialCommandGroup{
    public ClimbCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
            
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE),
                    new SetArmTarget(armSubsystem, Arm.ARM_CLIMB_POSE)
                ),
                new SequentialCommandGroup(
                    new SetArmTarget(armSubsystem, Arm.ARM_CLIMB_POSE),
                    new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE)
                ),
                () -> intakeSubsystem.getWristAngleRads() > Intake.FOLDED_POSE_INTERNAL + Constants.MARGIN_OF_ERROR_RADS
            ),

            new WaitUntilCommand(() -> false) // This is here otherwise command ends and robot will go to traverse position

        );
    }
}
