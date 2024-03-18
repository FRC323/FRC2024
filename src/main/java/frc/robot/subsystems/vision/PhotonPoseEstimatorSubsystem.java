package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.drive.DriveSubsystem;

public class PhotonPoseEstimatorSubsystem extends SubsystemBase{ 
    private static PhotonCamera frontRightPhotonCamera = new PhotonCamera("Front Right Camera");
    private static PhotonCamera frontLeftPhotonCamera = new PhotonCamera("Front Left Camera");
    private static PhotonCamera backPhotonCamera = new PhotonCamera("Back Camera");


    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPoseEstimator backPoseEstimator = 
        new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backPhotonCamera, Vision.BACK_CAMERA_TO_ROBOT);
    private PhotonPoseEstimator frontRightEstimator = 
        new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontRightPhotonCamera, Vision.FRONT_RIGHT_CAMERA_TO_ROBOT);
    private PhotonPoseEstimator frontLeftEstimator =
         new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontLeftPhotonCamera, Vision.FRONT_LEFT_CAMERA_TO_ROBOT);


    private SwerveDrivePoseEstimator poseEstimator;
    private DriveSubsystem driveSubsystem;

    //Todo: Move to Constants.java and make individual values for each camera
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    public PhotonPoseEstimatorSubsystem(DriveSubsystem driveSubsystem){
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
        addEstimation(backPoseEstimator, backPhotonCamera.getLatestResult().getTimestampSeconds(), visionMeasurementStdDevs);
        addEstimation(frontRightEstimator, frontRightPhotonCamera.getLatestResult().getTimestampSeconds(), visionMeasurementStdDevs);
        addEstimation(frontLeftEstimator, frontLeftPhotonCamera.getLatestResult().getTimestampSeconds(), visionMeasurementStdDevs);
    }

    public void updateOdometry(){
        driveSubsystem.resetYawToAngle(poseEstimator.getEstimatedPosition().getRotation().rotateBy(new Rotation2d(Math.PI)).getDegrees());
        driveSubsystem.resetOdometry(poseEstimator.getEstimatedPosition());
        System.out.println("Updated Odometry From Limelight");
    }
    
    public Pose2d getEstimatedPose2d(){
        return poseEstimator.getEstimatedPosition();
    }
        
    private void addEstimation(PhotonPoseEstimator estimator,double timestamp, Matrix<N3,N1> visionMeasurementStdDevs){
        var optionalEstimatedPose = estimator.update();
        if(optionalEstimatedPose.isEmpty()) return;
        
        //Checks if the pose difference is within 1 meter
        var differenceInTranslation = optionalEstimatedPose.get().estimatedPose.toPose2d().relativeTo(poseEstimator.getEstimatedPosition());
        if(
            Math.abs(differenceInTranslation.getX()) > 1.0
            || Math.abs(differenceInTranslation.getY()) > 1.0
        ) return;

        //make sure est pose is within field boundaries
        if (optionalEstimatedPose.get().estimatedPose.toPose2d().getX() > aprilTagFieldLayout.getFieldLength())
            return;

        if (optionalEstimatedPose.get().estimatedPose.toPose2d().getY() > aprilTagFieldLayout.getFieldWidth())
            return;

        poseEstimator.addVisionMeasurement(optionalEstimatedPose.get().estimatedPose.toPose2d(),timestamp,visionMeasurementStdDevs);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("XPose: ",() -> poseEstimator.getEstimatedPosition().getX(), null);
        builder.addDoubleProperty("YPose: ", () -> poseEstimator.getEstimatedPosition().getY(), null);
        builder.addDoubleProperty("Rotation: ", () -> poseEstimator.getEstimatedPosition().getRotation().getRadians(), null);
    }
}
