package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.LimelightOptions.VisionMode;

public class Limelight {
    private final String _name;
    
    public Limelight() {
        this._name = LimelightHelpers.sanitizeName("limelight");
        setCameraMode(LimelightOptions.VisionMode.VisionProcessing);
    }
    
    public LimelightCaptureDetail capture() {
        return new LimelightCaptureDetail(
            LimelightHelpers.getFiducialID(this._name),
            LimelightHelpers.getTV(this._name),
            LimelightHelpers.getTX(this._name), 
            LimelightHelpers.getTY(this._name),
            LimelightHelpers.getLatency_Capture(this._name),
            getBotPoseFromAlliance(),
            LimelightHelpers.getBotPose2d(this._name),
            LimelightHelpers.getBotPose_TargetSpace(this._name),
            LimelightHelpers.getCameraPose_TargetSpace(this._name),
            LimelightHelpers.getTargetPose_RobotSpace(this._name),
            LimelightHelpers.getTargetPose_CameraSpace(this._name));
    }
    
    public Pose2d getBotPoseFromAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            var alliance = DriverStation.getAlliance().get();
            return alliance == Alliance.Blue ? 
                LimelightHelpers.getBotPose2d_wpiBlue(this._name) : 
                LimelightHelpers.getBotPose2d_wpiRed(this._name);
        }
        //remove botpose cuz not sure if causing issues
        System.out.println("No alliance info available for botpose");
        throw new RuntimeException("No alliance info available for botpose");
    }

    public void setCameraMode(LimelightOptions.VisionMode camMode) {
        if (camMode == VisionMode.DriverCamera)
            LimelightHelpers.setCameraMode_Driver(this._name);
        else
            LimelightHelpers.setCameraMode_Processor(this._name);
    }

    public record LimelightCaptureDetail 
    (
        double aprilTagId, 
        boolean hasTarget,
        double xOffset,
        double yOffset,
        double latency,
        Pose2d botpose_alliance,
        Pose2d botpose,
        double[] robotpose_targetspace,
        double[] camerapose_targetspace,
        double[] targetpose_robotspace,
        double[] targetpose_cameraspace
    ) {}
}