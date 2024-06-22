// package frc.robot.subsystems.vision;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.robot.subsystems.vision.LimelightOptions.VisionMode;
// import frc.robot.Constants.Vision;

// public class Limelight {
//     public final static String _name = LimelightHelpers.sanitizeName("limelight");
    
//     public Limelight() {
//         // this._name = LimelightHelpers.sanitizeName("limelight");
//         setCameraMode(LimelightOptions.VisionMode.VisionProcessing);
//         // LimelightHelpers.setCameraPose_RobotSpace(
//         //     _name,
//         //     -Vision.BACK_CAMERA_TO_ROBOT.getX(),
//         //     Vision.BACK_CAMERA_TO_ROBOT.getY(),
//         //     Vision.BACK_CAMERA_TO_ROBOT.getZ(),
//         //     Vision.BACK_CAMERA_TO_ROBOT.getRotation().getX(),
//         //     Vision.BACK_CAMERA_TO_ROBOT.getRotation().getY(),
//         //     -Vision.BACK_CAMERA_TO_ROBOT.getRotation().getZ());
//     }
    
//     public LimelightCaptureDetail capture() {
//         return new LimelightCaptureDetail(
//             LimelightHelpers.getFiducialID(this._name),
//             LimelightHelpers.getTV(this._name),
//             LimelightHelpers.getTX(this._name), 
//             LimelightHelpers.getTY(this._name),
//             LimelightHelpers.getLatency_Capture(this._name),
//             LimelightHelpers.getBotPose2d(this._name),
//             LimelightHelpers.getBotPose2d_wpiBlue(this._name),
//             LimelightHelpers.getBotPose2d_wpiRed(this._name),
//             LimelightHelpers.getBotPose_TargetSpace(this._name),
//             LimelightHelpers.getCameraPose_TargetSpace(this._name),
//             LimelightHelpers.getTargetPose_RobotSpace(this._name),
//             LimelightHelpers.getTargetPose_CameraSpace(this._name));
//     }
    
//     // public Pose2d getBotPoseFromAlliance() {
//     //     if (DriverStation.getAlliance().isPresent()) {
//     //         var alliance = DriverStation.getAlliance().get();
//     //         return alliance == Alliance.Blue ? 
//     //             LimelightHelpers.getBotPose2d_wpiBlue(this._name) : 
//     //             LimelightHelpers.getBotPose2d_wpiRed(this._name);
//     //     }
//     //     //remove botpose cuz not sure if causing issues
//     //     throw new RuntimeException("No alliance info available for botpose");
//     // }

//     public void setCameraMode(LimelightOptions.VisionMode camMode) {
//         if (camMode == VisionMode.DriverCamera)
//             LimelightHelpers.setCameraMode_Driver(this._name);
//         else
//             LimelightHelpers.setCameraMode_Processor(this._name);
//     }

//     public record LimelightCaptureDetail 
//     (
//         double aprilTagId, 
//         boolean hasTarget,
//         double xOffset,
//         double yOffset,
//         double latency,
//         Pose2d botpose,
//         Pose2d botpose_blue,
//         Pose2d botpose_red,
//         double[] robotpose_targetspace,
//         double[] camerapose_targetspace,
//         double[] targetpose_robotspace,
//         double[] targetpose_cameraspace
//     ) {}
// }
