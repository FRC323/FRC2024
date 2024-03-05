package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.ShotState;

public class AlignWhileDriving2 extends SequentialCommandGroup{
    private ShotState shotState;
    
    private PIDController rotController = new PIDController(
        0.2,
        0.0,
        0.0
    );

    public AlignWhileDriving2(
        DriveSubsystem driveSubsystem,
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem,
        VisionSubsystem visionSubsystem,
        PoseEstimatorSubsystem poseEstimatorSubsystem,
        DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vTheta
    ){
        rotController.enableContinuousInput(0, Math.PI * 2);

        addCommands(
            new SetIntakeTarget(intakeSubsystem, Intake.UNFOLDED_POSE - 0.5),
            new ParallelCommandGroup(
                new SetShooterSpeed(armSubsystem, shotState.get_shooterSpeed()),
                new SetArmTarget(armSubsystem, shotState.get_armAngle().getRadians()),
                new ScheduleCommand(
                    new RunCommand(( () ->
                        driveSubsystem.drive(
                            vx.getAsDouble(),
                            vy.getAsDouble(),
                            shotState == null ?
                                vTheta.getAsDouble()
                                : rotController.calculate(
                                    driveSubsystem.getRobotPose2d().getRotation().getRadians()
                                )
                            ,
                            true
                        )
                    ),driveSubsystem)
                )
            ),
            new SetShooterSpeed(armSubsystem, 0),
            new SetFeederSpeed(armSubsystem, 0)
        );
    

    }
}
