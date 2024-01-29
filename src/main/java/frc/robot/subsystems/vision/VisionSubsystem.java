package frc.robot.subsystems.vision;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Limelight.LimeLightCaptureDetail;

public class VisionSubsystem extends SubsystemBase {
    private final Limelight _limelight = new Limelight();
    private LimeLightCaptureDetail _limelightCapture = new LimeLightCaptureDetail();

    public VisionSubsystem() {}

    @Override
    public void periodic() {
        _limelightCapture = _limelight.capture();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addIntegerProperty("April Tag Id", _limelightCapture::getAprilTagId , null);
        builder.addBooleanProperty("Found Target", _limelightCapture::getHasTarget, null);
        builder.addDoubleProperty("Target xOffset", _limelightCapture::getXOffset, null);
        builder.addDoubleProperty("Target yOffset",  _limelightCapture::getYOffset, null);
        builder.addIntegerProperty("Camera Mode", _limelightCapture::getCameraMode, null);
        builder.addDoubleProperty("Calc Dist To Target", _limelightCapture::getCalculatedDistanceToTarget, null);
    }
}
