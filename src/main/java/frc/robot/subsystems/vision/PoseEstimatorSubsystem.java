package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ShotState;

public class PoseEstimatorSubsystem extends SubsystemBase{ 
    // private static PhotonCamera frontRightPhotonCamera = new PhotonCamera("Front Right Camera");
    // private static PhotonCamera frontLeftPhotonCamera = new PhotonCamera("Front Left Camera");
    public static PhotonCamera backPhotonCamera = new PhotonCamera("BackCamera");


    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPoseEstimator backPoseEstimator = 
        new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backPhotonCamera, Vision.BACK_CAMERA_TO_ROBOT);
    // private PhotonPoseEstimator frontRightEstimator = 
    //     new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontRightPhotonCamera, Vision.FRONT_RIGHT_CAMERA_TO_ROBOT);
    // private PhotonPoseEstimator frontLeftEstimator =
    //      new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontLeftPhotonCamera, Vision.FRONT_LEFT_CAMERA_TO_ROBOT);


    private SwerveDrivePoseEstimator poseEstimator;
    private DriveSubsystem driveSubsystem;

    //Todo: Move to Constants.java and make individual values for each camera
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private static ShotState shotState = new ShotState(new Rotation2d(0.0), new Rotation2d(0.0), 0.0); 

    private MedianFilter headingLimiter = new MedianFilter(10);
    private MedianFilter armAngleLimiter = new MedianFilter(10);
    private MedianFilter shooterSpeedLimiter = new MedianFilter(10);



    public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem){
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(driveSubsystem.getGyroYaw()),
            driveSubsystem.getModulePositions(),
            driveSubsystem.getPose(),
            stateStdDevs,
            visionMeasurementStdDevs);

        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void periodic(){
        //Todo: Fina camera latency
        update(backPhotonCamera, visionMeasurementStdDevs);
        // addEstimation(frontRightEstimator, frontRightPhotonCamera.getLatestResult().getTimestampSeconds(), visionMeasurementStdDevs);
        // addEstimation(frontLeftEstimator, frontLeftPhotonCamera.getLatestResult().getTimestampSeconds(), visionMeasurementStdDevs);

        if(!DriverStation.isAutonomous()){
            driveSubsystem.resetOdometry(poseEstimator.getEstimatedPosition());
            // _driveSubsystem.resetYawToAngle(capture.botpose_blue().getRotation().rotateBy(new Rotation2d(Math.PI)).getDegrees());
          }
    }

    public void updateOdometry(){
        driveSubsystem.resetYawToAngle(poseEstimator.getEstimatedPosition().getRotation().rotateBy(new Rotation2d(Math.PI)).getDegrees());
        driveSubsystem.resetOdometry(poseEstimator.getEstimatedPosition());
        System.out.println("Updated Odometry From Limelight");
    }
    
    public Pose2d getEstimatedPose2d(){
        return poseEstimator.getEstimatedPosition();
    }
        
    private void update(PhotonCamera camera, Matrix<N3,N1> visionMeasurementStdDevs){
        poseEstimator.update(Rotation2d.fromDegrees(driveSubsystem.getGyroYaw()), driveSubsystem.getModulePositions());

        var result = camera.getLatestResult();
        if(!result.hasTargets()){
            return;
        }

        //Todo: Use Multi tag readings
        
        var imageCaptureTime = result.getTimestampSeconds();
        var camToTarget = result.getBestTarget().getBestCameraToTarget();
        var camPose = aprilTagFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().transformBy(camToTarget.inverse());
        var robotPose = camPose.transformBy(Constants.Vision.BACK_CAMERA_TO_ROBOT);

        //Checks if the pose difference is within 1 meter
        // var differenceInTranslation = multiTagResults.best. (poseEstimator.getEstimatedPosition());
        // if(
        //     Math.abs(differenceInTranslation.getX()) > 1.0
        //     || Math.abs(differenceInTranslation.getY()) > 1.0
        // ) return;

        // //make sure est pose is within field boundaries
        // if (multiTagResults.get().estimatedPose.toPose2d().getX() > aprilTagFieldLayout.getFieldLength())
        //     return;

        // if (multiTagResults.get().estimatedPose.toPose2d().getY() > aprilTagFieldLayout.getFieldWidth())
        //     return;

        poseEstimator.addVisionMeasurement(robotPose.toPose2d(),imageCaptureTime,visionMeasurementStdDevs);

        this.computeShotState(driveSubsystem, getEstimatedPose2d());
    }

    public double get_shooterSpeed(){
        // shooterSpeedLimiter.calculate(shotState.get_shooterSpeed());
        return shotState.get_shooterSpeed();
    }

    public double get_armAngle(){
        return armAngleLimiter.calculate(shotState.get_armAngle().getRadians());
    }

    public double get_heading(){
        return headingLimiter.calculate(shotState.get_heading().getRadians());
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
        var dt = 0.75;

        this.shotState =  ShotState.computedFromPose(
            shotTarget,
            // rangeToTarget,
            robotPose,
            robotVelocity,
            dt
        );
    }
   
    public boolean isShotStateValid(){
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("XPose: ",() -> poseEstimator.getEstimatedPosition().getX(), null);
        builder.addDoubleProperty("YPose: ", () -> poseEstimator.getEstimatedPosition().getY(), null);
        builder.addDoubleProperty("Rotation: ", () -> poseEstimator.getEstimatedPosition().getRotation().getRadians(), null);
        builder.addDoubleProperty("ShotState Arm Angle", () -> shotState.get_armAngle().getDegrees() , null);
    }
}
