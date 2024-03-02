package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
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
        if(!limelightCapture.isPresent()) return;
        var capture =  limelightCapture.get();
        limelightCapture = _visionSubsystem.getLimelightCapture();
        double currentTimestamp = getTimestampSeconds(capture.latency());
        _poseEstimator.addVisionMeasurement(capture.botpose_alliance(), currentTimestamp);
       _poseEstimator.updateWithTime(currentTimestamp,new Rotation2d(_driveSubsystem.getGyroYaw()), _driveSubsystem.getModulePositions());

        // if(limelightCapture.hasTarget()){
        //     _driveSubsystem.resetOdometry(_poseEstimator.getEstimatedPosition());
        // }
    }

    public void updateOdometry(){
        if(!limelightCapture.isPresent()) return;
        var capture = limelightCapture.get();
        if(capture.hasTarget()){
            // _driveSubsystem.resetOdometry(_poseEstimator.getEstimatedPosition());
            // _driveSubsystem.resetYawToAngle(limelightCapture.botpose_alliance().getRotation().getRadians() + Math.PI);
            // _driveSubsystem.resetYawToAngle(limelightCapture.botpose_alliance().getRotation().getRadians());
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("XPose: ",() -> _poseEstimator.getEstimatedPosition().getX(), null);
        builder.addDoubleProperty("YPose: ", () -> _poseEstimator.getEstimatedPosition().getY(), null);
        builder.addDoubleProperty("Rotation: ", () -> _poseEstimator.getEstimatedPosition().getRotation().getRadians(), null);

    }

}
