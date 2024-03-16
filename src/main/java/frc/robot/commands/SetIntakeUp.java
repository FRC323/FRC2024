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

public class SetIntakeUp extends SequentialCommandGroup{
    public SetIntakeUp(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
            //Checks if intake is inside robot
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SetArmTarget(armSubsystem, Arm.ARM_INTAKE_UNFOLDING_POSE),
                    new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE)
                ),
                new InstantCommand(),
                () -> intakeSubsystem.getWristAngleRads() < (Intake.FOLDED_POSE - Constants.MARGIN_OF_ERROR_RADS)
            ),

            //Checks if intake is locked and arm is up
            new ConditionalCommand(
                new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE),
                new InstantCommand(),
                () -> intakeSubsystem.getWristAngleRads() > Intake.FOLDED_POSE - Constants.MARGIN_OF_ERROR_RADS
                    && intakeSubsystem.getWristAngleRads() < Intake.SHOOTING_POSE
                    && !(armSubsystem.getArmAngleRads() > Arm.ARM_DOWN_POSE - Constants.MARGIN_OF_ERROR_RADS)
            ),

            new SetArmTarget(armSubsystem, Arm.ARM_DOWN_POSE),
            new SetIntakeTarget(intakeSubsystem, Intake.FOLDED_POSE)            
        );
    }
}
