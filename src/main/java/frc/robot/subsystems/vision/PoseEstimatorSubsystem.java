package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private final DriveSubsystem _driveSubsystem;
    private final VisionSubsystem _visionSubsystem;
    private final SwerveDrivePoseEstimator _poseEstimator;

    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

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

    @Override
    public void periodic() {
        var limelightCapture = _visionSubsystem.getLimelightCapture();
        double currentTimestamp = getTimestampSeconds(limelightCapture.latency());
       _poseEstimator.addVisionMeasurement(limelightCapture.botpose_alliance(), currentTimestamp);
    
       _poseEstimator.updateWithTime(currentTimestamp,new Rotation2d(_driveSubsystem.getGyroYaw()), _driveSubsystem.getModulePositions());

        if(limelightCapture.hasTarget()){
            _driveSubsystem.resetOdometry(_poseEstimator.getEstimatedPosition());
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
