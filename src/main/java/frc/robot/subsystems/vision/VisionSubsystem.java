package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private Limelight _limelight;

    public VisionSubsystem() {
        this._limelight = new Limelight();
    }

    @Override
    public void periodic() {
        var frame = _limelight.readFrame();
    }
}
