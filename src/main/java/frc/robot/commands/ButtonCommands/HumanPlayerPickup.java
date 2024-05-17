package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Feeder;
import frc.robot.Constants.Intake;
import frc.robot.commands.Procedures.AdjustFeederNote;
import frc.robot.commands.Procedures.CheckIntakeGotoOut;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HumanPlayerPickup extends SequentialCommandGroup{
    public HumanPlayerPickup(IntakeSubsystem intake,ArmSubsystem armSubsystem, FeederSubsystem feederSubsystem){
        addCommands(
            new CheckIntakeGotoOut(armSubsystem, intake, Intake.SHOOTING_POSE),
            new SetIntakeTarget(intake, Intake.SHOOTING_POSE),
            new ParallelCommandGroup(
                new SetArmTarget(armSubsystem, Arm.ARM_HUMAN_PLAYER_POSE),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> armSubsystem.getArmAngleRads() <  Arm.ARM_INTAKE_UNFOLDING_POSE),
                    new SetIntakeTarget(intake, Intake.FOLDED_POSE_INTERNAL)
                )
            ),
            new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_INTAKE_SPEED),
            new WaitUntilCommand(feederSubsystem::isIntialBeamTriggered),
            new SetFeederSpeed(feederSubsystem, Feeder.FEEDER_STOPED_SPEED)
            // new AdjustFeederNote(feederSubsystem)
        );
    }
}
