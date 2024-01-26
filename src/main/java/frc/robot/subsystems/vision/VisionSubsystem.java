package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Limelight.LimeLightCaptureDetail;

public class VisionSubsystem extends SubsystemBase {
    private Limelight _limelight;

    public VisionSubsystem() {
        this._limelight = new Limelight();
    }

    public LimeLightCaptureDetail getLimelightCapture() {
        return _limelight.capture();
    }

    @Override
    public void periodic() {}
}
