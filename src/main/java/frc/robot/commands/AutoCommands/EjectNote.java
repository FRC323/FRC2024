package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Feeder;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.commands.Procedures.CheckIntakeGotoOut;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetIntakeSpeed;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EjectNote extends SequentialCommandGroup{
    public EjectNote(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem){
        addCommands(
            new CheckIntakeGotoOut(armSubsystem, intakeSubsystem, Intake.UNFOLDED_POSE),
            new SetIntakeTarget(intakeSubsystem, Intake.UNFOLDED_POSE),
            new SetArmTarget(armSubsystem, Arm.ARM_HANDOFF_POSE),
            new SetIntakeSpeed(intakeSubsystem, Intake.INTAKE_SPEED),
            new SetFeederSpeed(feederSubsystem, Feeder.FEEDER_INTAKE_SPEED),
            new SetShooterSpeed(shooterSubsystem, Shooter.EJECT_SPEED)
        );
    }
}
