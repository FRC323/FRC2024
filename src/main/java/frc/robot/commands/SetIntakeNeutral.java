package frc.robot.commands;

import static frc.robot.Constants.MARGIN_OF_ERROR_RADS;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeNeutral extends SequentialCommandGroup{
    public SetIntakeNeutral(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
            //Checks if intake is inside robot
            new ConditionalCommand(
                new SetArmTarget(armSubsystem, Arm.ARM_INTAKE_UNFOLDING_POSE),
                new InstantCommand(),
                () -> intakeSubsystem.getWristAngleRads() < (Intake.FOLDED_POSE - Constants.MARGIN_OF_ERROR_RADS)
            ),

            new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE)
        );
    }
}
