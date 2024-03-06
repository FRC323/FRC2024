package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
        VisionSubsystem visionSubsystem,
        DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vTheta
    ){
        addCommands(
            // new SetIntakeUnfolded(intakeSubsystem, armSubsystem),
            new ConditionalCommand(
                new InstantCommand(), 
                new RunCommand( () ->
                    driveSubsystem.driveWithHeading(
                        vx.getAsDouble(),
                        vy.getAsDouble(),
                        Rotation2d.fromRadians(visionSubsystem.get_heading()),
                        true
                    )
                ),
                () -> !visionSubsystem.isShotStateValid()
            )
        );
    }
}
