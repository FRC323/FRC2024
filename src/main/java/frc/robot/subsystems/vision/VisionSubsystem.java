package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight.LimelightCaptureDetail;
import frc.robot.utils.ShotState;

public class VisionSubsystem extends SubsystemBase {
    private final Limelight _limelight = new Limelight();
    private static LimelightCaptureDetail _limelightCapture;

    private static PhotonCamera frontPhotonCamera = new PhotonCamera("Front Camera");
    private static PhotonCamera backPhotonCamera = new PhotonCamera("Back Camera");
    private static List<PhotonTrackedTarget> visionTargets;
    
    private SlewRateLimiter headingLimiter = new SlewRateLimiter(4.0 * Math.PI);
    private SlewRateLimiter armAngleLimiter = new SlewRateLimiter(4.0 * Math.PI);
    private SlewRateLimiter shooterSpeedLimiter = new SlewRateLimiter(1.0);



    public VisionSubsystem(){}

    public Optional<LimelightCaptureDetail> getLimelightCapture() {
        //TODO: Find a way to handle null limelight captures throughout the whol program
        if(_limelightCapture == null) return Optional.empty();
        return Optional.of(_limelightCapture);
    }
    
    public static OptionalDouble getTargetDistance(){
        if(_limelightCapture == null) return OptionalDouble.empty();
        double tagHeight = Constants.AprilTags.Speaker.HEIGHT;
        double lensHeight = Constants.Vision.LIMELIGHT_LENS_HEIGHT_INCHES;
        double goalAngleRads = Units.degreesToRadians(Constants.Vision.LIMELIGHT_MOUNT_ANGLE_DEGREES + _limelightCapture.yOffset());
        return OptionalDouble.of((tagHeight - lensHeight) / Math.tan(goalAngleRads));
    }

    public static Optional<Pose2d> getFieldPose(){
        if(_limelightCapture == null) return Optional.empty();
        var botpose = _limelightCapture.botpose_blue();
        if(botpose == null || !_limelightCapture.hasTarget()) return Optional.empty();
        return Optional.of(botpose);
    }
    
    public double getTimestampSeconds() {
        var latencyMillis = _limelightCapture != null ? _limelightCapture.latency() : 0.0;
        return Timer.getFPGATimestamp() - (latencyMillis / 1000d);
    }

    @Override
    public void periodic() {
        visionTargets = frontPhotonCamera.getLatestResult().getTargets();

        try {
            _limelightCapture = _limelight.capture();
        }catch(Exception e){    
            System.out.println("Limelight Exception: " + e.toString());
            // System.out.println("No alliance info available for botpose");
        }

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("April Tag Id", () -> _limelightCapture.aprilTagId() , null);
        builder.addBooleanProperty("LL Found Target", () -> _limelightCapture.hasTarget(), null);
        builder.addDoubleProperty("LL Target xOffset", () -> _limelightCapture.xOffset(), null);
        builder.addDoubleProperty("LL Target yOffset",  () -> _limelightCapture.yOffset(), null);
        builder.addDoubleArrayProperty("LL BotPose TargetSpace",  () -> _limelightCapture.robotpose_targetspace(), null);
        builder.addDoubleArrayProperty("LL CameraPose TargetSpace",  () -> _limelightCapture.camerapose_targetspace(), null);
        builder.addDoubleArrayProperty("LL TargetPose RobotSpace",  () -> _limelightCapture.targetpose_robotspace(), null);
        builder.addDoubleArrayProperty("LL TargetPose CameraSpace",  () -> _limelightCapture.targetpose_cameraspace(), null);
        builder.addDoubleProperty("Target Distance", () -> VisionSubsystem.getTargetDistance().orElse(-1.0),null);
    }
}
