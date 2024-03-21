package frc.robot.commands.Procedures;

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
            new CheckIntakeGotoOut(armSubsystem, intakeSubsystem,Intake.SHOOTING_POSE),
            new SetArmTarget(armSubsystem, Arm.ARM_DOWN_POSE),
            new SetIntakeTarget(intakeSubsystem, Intake.FOLDED_POSE)            
        );
    }
}
