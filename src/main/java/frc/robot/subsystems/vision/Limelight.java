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

    public FrameDetail readFrame() {
        var fi = new FrameDetail();
        fi.AprilTagId = (int) _dataTable.getEntry("tid").getInteger(-1);
        fi.HasTarget = _dataTable.getEntry("tv").getBoolean(false);
        fi.xOffset = _dataTable.getEntry("tx").getDouble(0);
        fi.yOffset = _dataTable.getEntry("ty").getDouble(0);
        return fi;
    }

    public void setCamMode(LimelightOptions.VisionMode camMode) {
        _dataTable.getEntry("camMode").setNumber(camMode.get());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
       //SMDB logging
    }

    public static class FrameDetail {
        public int AprilTagId = -1; //-1=no target
        public boolean HasTarget = false;
        public double xOffset = 0d;
        public double yOffset = 0d;

        public FrameDetail() {}
        public FrameDetail(int tagId, boolean hasTarget, double xoffset, double yoffset) {
            this.AprilTagId = tagId;
            this.HasTarget = hasTarget;
            this.xOffset = xoffset;
            this.yOffset = yoffset;
        }
    }
}
