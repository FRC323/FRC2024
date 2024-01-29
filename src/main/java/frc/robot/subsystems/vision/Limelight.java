package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Limelight {
    private NetworkTable _dataTable;
    private LimeLightCaptureDetail _captureDetail = new LimeLightCaptureDetail();
    
    public Limelight() {
        this._dataTable = NetworkTableInstance.getDefault().getTable("limelight");
        _dataTable.getEntry("pipeline").setNumber(Constants.Vision.APRIL_TAG_PIPELINE);
        setCameraMode(LimelightOptions.VisionMode.VisionProcessing);
    }
    
    public LimeLightCaptureDetail capture() {
        _captureDetail.AprilTagId = _dataTable.getEntry("tid").getInteger(0);
        _captureDetail.HasTarget = _dataTable.getEntry("tv").getInteger(0) == 1;
        _captureDetail.xOffset = _dataTable.getEntry("tx").getDouble(0);
        _captureDetail.yOffset = _dataTable.getEntry("ty").getDouble(0);
        _captureDetail.CameraMode = _dataTable.getEntry("camMode").getInteger(0);
        _captureDetail.CalculatedDistanceToTarget = calculateDistanceToTarget();
        return _captureDetail;
    }

    public void setCameraMode(LimelightOptions.VisionMode camMode) {
        _dataTable.getEntry("camMode").setNumber(camMode.get());
    }

    private double calculateDistanceToTarget() {
        double speakerGoalHeight = Constants.Vision.SPEAKER_GOAL_HEIGHT_INCHES;
        double lensHeight = Constants.Vision.LIMELIGHT_LENS_HEIGHT_INCHES;
        double goalAngleRads = Units.degreesToRadians(Constants.Vision.LIMELIGHT_MOUNT_ANGLE_DEGRESS + _captureDetail.yOffset);
        return (speakerGoalHeight - lensHeight) / Math.tan(goalAngleRads);
    }

    public static class LimeLightCaptureDetail {
        public long AprilTagId = -1; //-1=no target
        public boolean HasTarget = false;
        public double xOffset = 0d;
        public double yOffset = 0d;
        public long CameraMode = 0;
        public double CalculatedDistanceToTarget = 0.0;

        public long getAprilTagId() {
            return AprilTagId;
        }

        public boolean getHasTarget() {
            return HasTarget;
        }

        public double getXOffset() {
            return xOffset;
        }

        public double getYOffset() {
            return yOffset;
        }

        public long getCameraMode() {
            return CameraMode;
        }

        public double getCalculatedDistanceToTarget() {
            return CalculatedDistanceToTarget;
        }
    }
}
