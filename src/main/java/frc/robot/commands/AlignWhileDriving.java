package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.ShotState;

public class AlignWhileDriving extends SequentialCommandGroup{
    public AlignWhileDriving(
        DriveSubsystem driveSubsystem,
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem,
        VisionSubsystem visionSubsystem,
        PoseEstimatorSubsystem poseEstimatorSubsystem,
        DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vTheta
    ){
        addCommands(
            new SetIntakeTarget(intakeSubsystem, Intake.UNFOLDED_POSE - 0.5),
            new ConditionalCommand(
                new ScheduleCommand(
                    new RunCommand( () ->
                        driveSubsystem.drive(
                            vx.getAsDouble(),
                            vy.getAsDouble(),
                            vTheta.getAsDouble(),
                            true
                        )
                    )
                ),
                new ParallelCommandGroup(
                    new SetShooterSpeed(armSubsystem, VisionSubsystem.getShotState().get().get_shooterSpeed()),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(intakeSubsystem::wristIsAtTarget),
                        new SetArmTarget(armSubsystem, VisionSubsystem.getShotState().get().get_armAngle().getRadians())
                    ),
                    new ScheduleCommand(
                        new RunCommand( () ->
                            driveSubsystem.driveWithHeading(
                                vx.getAsDouble(),
                                vy.getAsDouble(),
                                VisionSubsystem.getShotState().get().get_heading(),
                                true
                            )
                        )
                    )
                ),
                () -> VisionSubsystem.getShotState().isPresent()
            ),
            new SetShooterSpeed(armSubsystem, 0),
            new SetFeederSpeed(armSubsystem, 0)
        );
    }
}
