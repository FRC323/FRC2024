package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Shooter;
import frc.robot.commands.Procedures.AdjustFeederNote;
import frc.robot.commands.Procedures.CheckIntakeGotoOut;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup{
    public ShootCommand(FeederSubsystem feederSubsystem,ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem){
        addCommands(
            new SetShooterSpeed(shooterSubsystem, () -> shooterSubsystem.getShootSpeedTarget()),
            // new SequentialCommandGroup(
            //     new WaitUntilCommand(() -> armSubsystem.getArmAngleRads() <= Arm.ARM_HANDOFF_POSE + Arm.AT_TARGET_TOLLERANCE),
            //     new AdjustFeederNote(feederSubsystem)
            // ),
            new ParallelRaceGroup(
                new ConditionalCommand(    
                    new WaitUntilCommand(shooterSubsystem::atShootSpeed),
                    new SequentialCommandGroup(
                        new SetShooterSpeed(shooterSubsystem,Shooter.SHOOTER_SPEED),
                        new WaitUntilCommand(shooterSubsystem::atShootSpeed)
                    ),
                    () -> shooterSubsystem.isRunning()
                ),
                new WaitCommand(2.5)
            ),
            new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEED_SHOOT_SPEED),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(feederSubsystem::isHoldingNote),
                    new WaitUntilCommand(() -> !feederSubsystem.isHoldingNote())
                ),
                new WaitCommand(5)
            )
        );
    }
}
