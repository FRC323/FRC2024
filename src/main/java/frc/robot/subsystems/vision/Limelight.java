package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Limelight implements Sendable {
    private NetworkTable _dataTable;
    
    public Limelight() {
        this._dataTable = NetworkTableInstance.getDefault().getTable("limelight");
        setCamMode(LimelightOptions.VisionMode.VisionProcessing);
    }

    public LimeLightCaptureDetail capture() {
        var fi = new LimeLightCaptureDetail();
        fi.AprilTagId = _dataTable.getEntry("tid").getInteger(-1);
        fi.HasTarget = _dataTable.getEntry("tv").getInteger(0) == 1;
        fi.xOffset = _dataTable.getEntry("tx").getDouble(0);
        fi.yOffset = _dataTable.getEntry("ty").getDouble(0);
        fi.CameraMode = _dataTable.getEntry("camMode").getInteger(0);
        return fi;
    }

    public void setCamMode(LimelightOptions.VisionMode camMode) {
        _dataTable.getEntry("camMode").setNumber(camMode.get());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        LimeLightCaptureDetail capture = capture();
        builder.setSmartDashboardType("Limelight");
        builder.addIntegerProperty("April Tag Id", () -> capture.AprilTagId, null);
        builder.addBooleanProperty("Found Target", () -> capture.HasTarget, null);
        builder.addDoubleProperty("Target xOffset", () -> capture.xOffset, null);
        builder.addDoubleProperty("Target yOffset", () -> capture.yOffset, null);
        builder.addIntegerProperty("Camera Mode", () -> capture.CameraMode, null);
    }

    public static class LimeLightCaptureDetail {
        public long AprilTagId = -1; //-1=no target
        public boolean HasTarget = false;
        public double xOffset = 0d;
        public double yOffset = 0d;
        public long CameraMode = 0;
    }
}
