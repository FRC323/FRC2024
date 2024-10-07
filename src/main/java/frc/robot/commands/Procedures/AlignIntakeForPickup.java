package frc.robot.commands.Procedures;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AlignIntakeForPickup extends SequentialCommandGroup {
    public AlignIntakeForPickup(
        VisionSubsystem visionSubsystem,
        DriveSubsystem driveSubsystem,
        DoubleSupplier vx, DoubleSupplier vy
    ) {
        addCommands(
            new RunCommand(() -> driveSubsystem.driveWithHeading(vx.getAsDouble(), vy.getAsDouble(), Rotation2d.fromDegrees(visionSubsystem.getYaw()), true))
        );
    }
}