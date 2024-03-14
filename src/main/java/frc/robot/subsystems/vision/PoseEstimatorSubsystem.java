package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight.LimelightCaptureDetail;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private final DriveSubsystem _driveSubsystem;
    private final VisionSubsystem _visionSubsystem;
    private final SwerveDrivePoseEstimator _poseEstimator;

    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private Optional<LimelightCaptureDetail> limelightCapture = Optional.empty();

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();

    public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this._driveSubsystem = driveSubsystem;
        this._visionSubsystem = visionSubsystem;

        _poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.DRIVE_KINEMATICS,
                Rotation2d.fromDegrees(_driveSubsystem.getGyroYaw()),
                _driveSubsystem.getModulePositions(),
                _driveSubsystem.getPose(),
                stateStdDevs,
                visionMeasurementStdDevs);
    }

    public double getTimestampSeconds(double latencyMillis) {
        return Timer.getFPGATimestamp() - (latencyMillis / 1000d);
    }

    public Pose2d getEstimatedPosition(){
        return _poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        limelightCapture = _visionSubsystem.getLimelightCapture();
        if(!limelightCapture.isPresent()) return;
        var capture =  limelightCapture.get();
        double currentTimestamp = getTimestampSeconds(capture.latency());
        // Actually do we only want to do this if we have multiple targets?
        if (capture.hasTarget()) {
          // Should this subtract the LL latency from the  timestamp?
          _poseEstimator.addVisionMeasurement(capture.botpose_blue(), currentTimestamp);
        }
       _poseEstimator.updateWithTime(currentTimestamp,new Rotation2d(_driveSubsystem.getGyroYaw()), _driveSubsystem.getModulePositions());

        if(capture.hasTarget() && !DriverStation.isAutonomous()){
            _driveSubsystem.resetOdometry(_poseEstimator.getEstimatedPosition());
            // _driveSubsystem.resetYawToAngle(capture.botpose_blue().getRotation().rotateBy(new Rotation2d(Math.PI)).getDegrees());
        }

        this._visionSubsystem.computeShotState(_driveSubsystem, getEstimatedPosition());
        // publisher.set(capture.botpose());
    }

    public void updateOdometry(){
        System.out.println("updating odometry");
        if(!limelightCapture.isPresent()) return;
        var capture = limelightCapture.get();
        System.out.println("Has Capture");
        // if(capture.aprilTagId() == 4.0){
            // System.out.println("Has Target");

            // _driveSubsystem.resetYawToAngle(limelightCapture.botpose_alliance().getRotation().getRadians() + Math.PI);
            _driveSubsystem.resetYawToAngle(capture.botpose_blue().getRotation().rotateBy(new Rotation2d(Math.PI)).getDegrees());
            _driveSubsystem.resetOdometry(_poseEstimator.getEstimatedPosition());
            System.out.println("Updated Odometry From Limelight");
        // }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("XPose: ",() -> _poseEstimator.getEstimatedPosition().getX(), null);
        builder.addDoubleProperty("YPose: ", () -> _poseEstimator.getEstimatedPosition().getY(), null);
        builder.addDoubleProperty("Rotation: ", () -> _poseEstimator.getEstimatedPosition().getRotation().getRadians(), null);

    }

}
