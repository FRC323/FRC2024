package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.Procedures.CheckIntakeGotoOut;
import frc.robot.commands.Procedures.SetIntakeUnfolded;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetIntakeSpeed;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OuttakeCommand extends SequentialCommandGroup{
    public OuttakeCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem){
        addCommands(
            new CheckIntakeGotoOut(armSubsystem, intakeSubsystem,Intake.UNFOLDED_POSE),
            new SetIntakeTarget(intakeSubsystem, Intake.UNFOLDED_POSE),
            new SetArmTarget(armSubsystem, Arm.ARM_OUTAKE_POSE),
            new SetIntakeSpeed(intakeSubsystem, Constants.Intake.OUTTAKE_SPEED),
            new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_REVERSE_SPEED),
            new SetShooterSpeed(shooterSubsystem, Constants.Shooter.REVERSE_SPEED),
            new WaitUntilCommand(() -> false)
        );
    }
}
