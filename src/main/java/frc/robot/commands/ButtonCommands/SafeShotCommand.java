package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.commands.Procedures.AdjustFeederNote;
import frc.robot.commands.Procedures.CheckIntakeGotoOut;
import frc.robot.commands.Procedures.SetIntakeFoldedInternal;
import frc.robot.commands.Procedures.SetIntakeUp;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SafeShotCommand extends SequentialCommandGroup{
    public SafeShotCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem){
        addCommands(

            //new WaitUntilCommand(() -> feederSubsystem.isHoldingNote()),

            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SetIntakeTarget(intakeSubsystem, Intake.SHOOTING_POSE),
                    new SetArmTarget(armSubsystem, Arm.ARM_SAFE_ZONE_SHOT)
                ),
                new SetArmTarget(armSubsystem, Arm.ARM_SAFE_ZONE_SHOT),
                () -> intakeSubsystem.getWristAngleRads() > Intake.FOLDED_POSE_INTERNAL + Constants.MARGIN_OF_ERROR_RADS
            ),

            new WaitUntilCommand(() -> (shooterSubsystem.isRunning())),

            new WaitUntilCommand(() -> !(shooterSubsystem.isRunning())),

            new SetIntakeFoldedInternal(intakeSubsystem, armSubsystem, feederSubsystem)
        );
    }
}
