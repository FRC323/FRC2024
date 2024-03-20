package frc.robot.commands.Procedures;

import static frc.robot.Constants.MARGIN_OF_ERROR_RADS;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class CheckIntakeGotoOut extends SequentialCommandGroup{
    public CheckIntakeGotoOut(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, double position){
        addCommands(
            //Checks if intake is inside robot
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SetArmTarget(armSubsystem, Arm.ARM_INTAKE_UNFOLDING_POSE),
                    new SetIntakeTarget(intakeSubsystem, position)
                ),
                new InstantCommand(),
                () -> intakeSubsystem.getWristAngleRads() < (Intake.FOLDED_POSE - Constants.MARGIN_OF_ERROR_RADS)
            )
        );
    }
}
