package frc.robot.subsystems.vision;

import java.beans.Visibility;
import java.util.Optional;
import java.util.OptionalDouble;

import javax.swing.text.html.Option;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ShotState;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private final SwerveDrivePoseEstimator poseEstimator;

    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    //private PhotonVision camera = new PhotonVision(Vision.PV_CAMERA_NAME, Vision.BACK_CAMERA_TO_ROBOT);
    private PhotonCamera camera = new PhotonCamera(Vision.PV_CAMERA_NAME);
    private AprilTagFieldLayout AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPoseEstimator estimator = new PhotonPoseEstimator(AprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Vision.BACK_CAMERA_TO_ROBOT);
    private Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
    //private Optional<LimelightCaptureDetail> limelightCapture = Optional.empty();

    //private final Limelight limelight = new Limelight();
    private boolean hasVisionTarget = false;
    
    private LinearFilter xFilter = LinearFilter.singlePoleIIR(0.2,0.02); //new SlewRateLimiter(1.0);
    private LinearFilter yFilter = LinearFilter.singlePoleIIR(0.2,0.02);// new SlewRateLimiter(1.0);
    private LinearFilter rotFilter = LinearFilter.singlePoleIIR(0.2,0.02);// new SlewRateLimiter(Math.PI);

    private static ShotState shotState = new ShotState(new Rotation2d(0.0), new Rotation2d(0.0), 0.0); 

   // private Pose2d estimatedPose = new Pose2d();

    public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.DRIVE_KINEMATICS,
                Rotation2d.fromDegrees(driveSubsystem.getGyroYaw()),
                driveSubsystem.getModulePositions(),
                driveSubsystem.getPose(),
                stateStdDevs,
                visionMeasurementStdDevs);
    }

    public Pose2d getEstimatedPosition(){
        return poseEstimator.getEstimatedPosition();
    }

    public double getTimestampSeconds(double latencyMillis) {
        return Timer.getFPGATimestamp() - (latencyMillis / 1000d);
    }

    public Pose2d filterVisionPose(Pose2d estimatedPose){
        var x = xFilter.calculate(estimatedPose.getX());
        var y = yFilter.calculate(estimatedPose.getY());
        var rot = rotFilter.calculate(estimatedPose.getRotation().getRadians());
        return new Pose2d(
            x,y,new Rotation2d(rot)
        );
    }

    @Override
    public void periodic() {
        estimatedPose = estimator.update();
        hasVisionTarget = false;

        if (camera.getLatestResult().hasTargets()) {
            if (estimatedPose != null && estimatedPose.isPresent()) {
                poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), camera.getLatestResult().getTimestampSeconds());
            }

            hasVisionTarget = true;
        }

        poseEstimator.updateWithTime(camera.getLatestResult().getTimestampSeconds(), new Rotation2d(driveSubsystem.getGyroYaw()), driveSubsystem.getModulePositions());

        //estimatedPose = filterVisionPose(poseEstimator.getEstimatedPosition());
        if (estimatedPose != null && estimatedPose.isPresent())
            this.computeShotState(driveSubsystem, estimatedPose.get().estimatedPose.toPose2d());
    }

    public boolean updateOdometry(){
        System.out.println("updating odometry");
        if (!camera.getLatestResult().hasTargets()) return false;
        driveSubsystem.resetYawToAngle(Rotation2d.fromDegrees(camera.getLatestResult().getBestTarget().getYaw()).plus(new Rotation2d(Math.PI)).getDegrees());
        driveSubsystem.resetOdometry(poseEstimator.getEstimatedPosition());
        System.out.println("Updated Odometry From PV");
        return true;
    }

    public Double getTargetDistance(){
      return PhotonUtils.calculateDistanceToTargetMeters(Constants.Vision.LIMELIGHT_LENS_HEIGHT_METERS,
            Vision.SPEAKER_HEIGHT_METERS,
            Units.degreesToRadians(Constants.Vision.LIMELIGHT_MOUNT_ANGLE_DEGREES),
            Units.degreesToRadians(camera.getLatestResult().getBestTarget().getPitch()));
    }

    public static ShotState getShotState(){
        return shotState;
    }

    public static boolean isShotStateValid(){
        return true;
        //todo: make this actually check
    }

    public double get_shooterSpeed(){
        return shotState.get_shooterSpeed();
    }

    public double get_armAngle(){
        return shotState.get_armAngle().getRadians();
    }

    public double get_heading(){
        return shotState.get_heading().getRadians();
    }

    public void computeShotState(DriveSubsystem driveSubsystem,Pose2d robotPose){
        //Shot Target
        if(DriverStation.getAlliance().isEmpty()){
            return;
        } 
        var shotTarget = DriverStation.getAlliance().get() == Alliance.Red ? Vision.RED_SHOT_TARGET : Vision.BLUE_SHOT_TARGET;
    
        //Range To Target
        // var optionalRange = VisionSubsystem.getTargetDistance();
        // if(optionalRange.isEmpty()){
        //     var rangeToTarget = 
        // } 
        // var rangeToTarget = optionalRange.getAsDouble();

        //Robot Pose
        // var robotPose =  poseEstimatorSubsystem.getEstimatedPosition();

        //Robot Velocity
        var robotVelocity = this.driveSubsystem.getChassisSpeed();

        //dt (Todo: find actual dt)
        var dt = DriverStation.isAutonomous() ? 0.0 : 0.5;

        this.shotState =  ShotState.computedFromPose(
            shotTarget,
            // rangeToTarget,
            robotPose,
            robotVelocity,
            dt
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("XPose",() -> poseEstimator.getEstimatedPosition().getX(), null);
        builder.addDoubleProperty("YPose", () -> poseEstimator.getEstimatedPosition().getY(), null);

       // if (camera.getEstimatedPose() != null && camera.getEstimatedPose().isPresent()) {
         //   builder.addDoubleProperty("PV Pose X", () -> camera.getEstimatedPose().get().estimatedPose.getX(), null);
         //   builder.addDoubleProperty("PV Pose Y", () -> camera.getEstimatedPose().get().estimatedPose.getY(), null);
        //}

       // builder.addDoubleProperty("Filtered X", () -> estimatedPose.getX(), null);
        builder.addDoubleProperty("Rotation", () -> poseEstimator.getEstimatedPosition().getRotation().getRadians(), null);

        builder.addDoubleProperty("Heading", () -> shotState.get_heading().getDegrees(), null);
        builder.addDoubleProperty("Arm Angle", () -> shotState.get_armAngle().getRadians(), null);

        builder.addDoubleProperty("Shooter Speed", () -> shotState.get_shooterSpeed(), null);

        builder.addBooleanProperty("HasVisionTarget",() -> hasVisionTarget, null);
    }

}
