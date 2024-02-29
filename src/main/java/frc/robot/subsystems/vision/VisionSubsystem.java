package frc.robot.subsystems.vision;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Limelight.LimelightCaptureDetail;

public class VisionSubsystem extends SubsystemBase {
    private final Limelight _limelight = new Limelight();
    private LimelightCaptureDetail _limelightCapture;

    public LimelightCaptureDetail getLimelightCapture() {
        return _limelightCapture;
    }

    @Override
    public void periodic() {
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
        builder.addDoubleProperty("LL Alliance Bot Pose X",  () -> _limelightCapture.botpose_alliance().getX(), null);
        builder.addDoubleProperty("LL Alliance Bot Pose Y",  () -> _limelightCapture.botpose_alliance().getY(), null);
        builder.addDoubleArrayProperty("LL BotPose TargetSpace",  () -> _limelightCapture.robotpose_targetspace(), null);
        builder.addDoubleArrayProperty("LL CameraPose TargetSpace",  () -> _limelightCapture.camerapose_targetspace(), null);
        builder.addDoubleArrayProperty("LL TargetPose RobotSpace",  () -> _limelightCapture.targetpose_robotspace(), null);
        builder.addDoubleArrayProperty("LL TargetPose CameraSpace",  () -> _limelightCapture.targetpose_cameraspace(), null);
    }
}
