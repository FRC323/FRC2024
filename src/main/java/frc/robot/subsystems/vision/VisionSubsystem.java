package frc.robot.subsystems.vision;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera _camera;
    private Transform3d _robotToCamera;

    public enum VisionMode {
        Driver,
        Detection
    }

    public void SetVisionMode(VisionMode mode) {
        _camera.setDriverMode(mode == VisionMode.Driver);
    }

    public VisionSubsystem(String cameraName, Transform3d robotToCamera) {
        _camera = new PhotonCamera(cameraName);
        _robotToCamera = robotToCamera;
    }

    public List<PhotonTrackedTarget> getVisibleObjects() {
        var results = _camera.getLatestResult();
        return results.targets;
    }

    public PhotonTrackedTarget getBestObject() {
        var results = _camera.getLatestResult();
        if (results.hasTargets())
            return results.getBestTarget();
        return null;
    }

    public double getYaw() {
        var best = getBestObject();
        return best != null ? best.getYaw() : 0;
    }

    public boolean hasTarget() {
        var best = getBestObject();
        return best != null;
    }

      @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Target Yaw", () -> getYaw(), null);
  }
}

