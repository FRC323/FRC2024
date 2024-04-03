package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.OptionalDouble;

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
import frc.robot.subsystems.vision.Limelight.LimelightCaptureDetail;
import frc.robot.utils.ShotState;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private final SwerveDrivePoseEstimator poseEstimator;

    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private Optional<LimelightCaptureDetail> limelightCapture = Optional.empty();

    private final Limelight limelight = new Limelight();
    
    private LinearFilter xFilter = LinearFilter.singlePoleIIR(0.2,0.02); //new SlewRateLimiter(1.0);
    private LinearFilter yFilter = LinearFilter.singlePoleIIR(0.2,0.02);// new SlewRateLimiter(1.0);
    private LinearFilter rotFilter = LinearFilter.singlePoleIIR(0.2,0.02);// new SlewRateLimiter(Math.PI);

    private static ShotState shotState = new ShotState(new Rotation2d(0.0), new Rotation2d(0.0), 0.0); 

    private Pose2d estimatedPose = new Pose2d();

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
        try {
            var tempCampture = limelight.capture();
            if(tempCampture == null) limelightCapture = Optional.empty();
            else limelightCapture = Optional.of(tempCampture);
        }catch(Exception e){    
            System.out.println("Limelight Exception: " + e.toString());
            // System.out.println("No alliance info available for botpose");
        }

        if(!limelightCapture.isPresent()) return;
        var capture =  limelightCapture.get();
        double currentTimestamp = getTimestampSeconds(capture.latency());
        // Actually do we only want to do this if we have multiple targets?
        if (capture.hasTarget()) {
          // Should this subtract the LL latency from the  timestamp?
          poseEstimator.addVisionMeasurement(capture.botpose_blue(), currentTimestamp);
        }
       poseEstimator.updateWithTime(currentTimestamp,new Rotation2d(driveSubsystem.getGyroYaw()), driveSubsystem.getModulePositions());

        if(capture.hasTarget() && !DriverStation.isAutonomous()){
            driveSubsystem.resetOdometry(poseEstimator.getEstimatedPosition());
            // _driveSubsystem.resetYawToAngle(capture.botpose_blue().getRotation().rotateBy(new Rotation2d(Math.PI)).getDegrees());
        }

        estimatedPose = filterVisionPose(poseEstimator.getEstimatedPosition());

        this.computeShotState(driveSubsystem, estimatedPose); 

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
            driveSubsystem.resetYawToAngle(capture.botpose_blue().getRotation().rotateBy(new Rotation2d(Math.PI)).getDegrees());
            driveSubsystem.resetOdometry(poseEstimator.getEstimatedPosition());
            System.out.println("Updated Odometry From Limelight");
        // }
    }

    public OptionalDouble getTargetDistance(){
        if(limelightCapture.isEmpty()) return OptionalDouble.empty();
        double tagHeight = Constants.AprilTags.Speaker.HEIGHT;
        double lensHeight = Constants.Vision.LIMELIGHT_LENS_HEIGHT_INCHES;
        double goalAngleRads = Units.degreesToRadians(Constants.Vision.LIMELIGHT_MOUNT_ANGLE_DEGREES + limelightCapture.get().yOffset());
        return OptionalDouble.of((tagHeight - lensHeight) / Math.tan(goalAngleRads));
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
        var dt = 0.5;

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
        builder.addDoubleProperty("Filtered X", () -> estimatedPose.getX(), null);
        builder.addDoubleProperty("Rotation", () -> poseEstimator.getEstimatedPosition().getRotation().getRadians(), null);

        builder.addDoubleProperty("Heading", () -> shotState.get_heading().getDegrees(), null);
        builder.addDoubleProperty("Arm Angle", () -> shotState.get_armAngle().getRadians(), null);

        builder.addDoubleProperty("Shooter Speed", () -> shotState.get_shooterSpeed(), null);
    }

}
