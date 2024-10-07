package frc.robot.commands.Procedures;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AlignWhileDriving extends SequentialCommandGroup{
    public AlignWhileDriving(
        DriveSubsystem driveSubsystem,
        DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vTheta
    ){
        addCommands(
            new RunCommand(() ->
                driveSubsystem.driveWithHeading(vx.getAsDouble(),vy.getAsDouble(),driveSubsystem.getShotState().get_heading(),true)
            )
        );
    }
}
