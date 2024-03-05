package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.Constants.PathFollowing;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.ShotState;

public class AlignWhileDriving extends Command{
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private VisionSubsystem visionSubsystem;
    private PoseEstimatorSubsystem poseEstimatorSubsystem;

    private DoubleSupplier vx,vy,vTheta;

    private ShotState shotState;
    
    private PIDController rotController = new PIDController(
        0.2,
        0.0,
        0.0
    );
    
    public AlignWhileDriving(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, VisionSubsystem visionSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem, DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vTheta){
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;

        this.vx = vx;
        this.vy = vy;
        this.vTheta = vTheta;

        rotController.enableContinuousInput(0, Math.PI * 2);

        addRequirements(
            driveSubsystem,
            armSubsystem,
            intakeSubsystem
        );

    }


    @Override
    public void execute(){

        updateShotState();

        if(shotState != null){
            rotController.setSetpoint(
                shotState.get_heading().getRadians()
            );
        }

        //Todo: Follow Heading
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
        );

        // intakeSubsystem.setTargetRads(Intake.UNFOLDED_POSE);

        // if(shotState != null && intakeSubsystem.getWristAngleRads() > Intake.UNFOLDED_POSE - 0.25){
        //     armSubsystem.setTargetRads(
        //         shotState.get_armAngle().getRadians()
        //     );
        // }

        // armSubsystem.setShooterSpeed(
        //     Constants.Arm.Shooter.SHOOTER_SPEED
        // );



    }
    
    private void updateShotState(){
        var optionalCapture = visionSubsystem.getLimelightCapture();
        if(optionalCapture.isEmpty()) return;
        var limelightCapture = optionalCapture.get();
        var shotTarget = new Translation2d(15.57,5.43);

        
        //Range To Target
        var optionalRange = VisionSubsystem.getTargetDistance();
        if(optionalRange.isEmpty()) return;
        var rangeToTarget = optionalRange.getAsDouble();

        //Robot Pose
        var robotPose =  driveSubsystem.getRobotPose2d();

        //Robot Velocity
        var robotVelocity = driveSubsystem.getChassisSpeed();

        //dt (Todo: find actual dt)
        var dt = 0.2;

        this.shotState = ShotState.computedFromPose(
            shotTarget,
            rangeToTarget,
            robotPose,
            robotVelocity,
            dt
        );
    }

    @Override
    public void end(boolean interupted){
        armSubsystem.setShooterSpeed(0);
        armSubsystem.setFeederSpeed(0);
        armSubsystem.setTargetRads(armSubsystem.getArmAngleRads());
    }

    @Override
    public void initSendable(SendableBuilder builder){
        super.initSendable(builder);

        builder.addDoubleProperty("Target Heading",() -> shotState == null ? Double.NaN : this.shotState.get_heading().getDegrees() ,null);
        builder.addDoubleProperty("Heading", () -> poseEstimatorSubsystem.getEstimatedPosition().getRotation().getDegrees(), null);
    }
}
