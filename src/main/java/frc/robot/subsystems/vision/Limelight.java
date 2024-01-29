package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Limelight {
    private NetworkTable _dataTable;
    private LimeLightCaptureDetail _captureDetail = new LimeLightCaptureDetail();
    
    public Limelight() {
        this._dataTable = NetworkTableInstance.getDefault().getTable(LimelightKeys.limelight);
        _dataTable.getEntry(LimelightKeys.pipeline).setNumber(Constants.Vision.APRIL_TAG_PIPELINE);
        setCameraMode(LimelightOptions.VisionMode.VisionProcessing);
    }
    
    public LimeLightCaptureDetail capture() {
        _captureDetail.setAprilTagId(_dataTable.getEntry(LimelightKeys.tid).getInteger(0));
        _captureDetail.setHasTarget(_dataTable.getEntry(LimelightKeys.tv).getInteger(0) == 1);
        _captureDetail.setXOffset(_dataTable.getEntry(LimelightKeys.tx).getDouble(0));
        _captureDetail.setYOffset(_dataTable.getEntry(LimelightKeys.ty).getDouble(0));
        _captureDetail.setCameraMode(_dataTable.getEntry(LimelightKeys.camMode).getInteger(0));
        _captureDetail.setCalculatedDistanceToTarget(calculateDistanceToTarget());
        return _captureDetail;
    }

    public void setCameraMode(LimelightOptions.VisionMode camMode) {
        _dataTable.getEntry(LimelightKeys.camMode).setNumber(camMode.get());
    }

    private double calculateDistanceToTarget() {
        double speakerGoalHeight = Constants.Vision.SPEAKER_GOAL_HEIGHT_INCHES;
        double lensHeight = Constants.Vision.LIMELIGHT_LENS_HEIGHT_INCHES;
        double goalAngleRads = Units.degreesToRadians(Constants.Vision.LIMELIGHT_MOUNT_ANGLE_DEGRESS + _captureDetail.getYOffset());
        return (speakerGoalHeight - lensHeight) / Math.tan(goalAngleRads);
    }

    public static class LimeLightCaptureDetail {
        private long _aprilTagId = -1; //-1=no target
        private boolean _hasTarget = false;
        private double _xOffset = 0d;
        private double _yOffset = 0d;
        private long _cameraMode = 0;
        private double _calculatedDistanceToTarget = 0.0;

        //#region "getters and setters"
        public long getAprilTagId() {
            return this._aprilTagId;
        }

        public boolean getHasTarget() {
            return this._hasTarget;
        }

        public double getXOffset() {
            return this._xOffset;
        }

        public double getYOffset() {
            return this._yOffset;
        }

        public long getCameraMode() {
            return this._cameraMode;
        }

        public double getCalculatedDistanceToTarget() {
            return this._calculatedDistanceToTarget;
        }

        public void setAprilTagId(long val) {
            this._aprilTagId = val;
        }

        public void setHasTarget(boolean val) {
            this._hasTarget = val;
        }

        public void setXOffset(double val) {
            this._xOffset = val;
        }

        public void setYOffset(double val) {
            this._yOffset = val;
        }

        public void setCameraMode(long val) {
            this._cameraMode = val;
        }

        public void setCalculatedDistanceToTarget(double val) {
            this._calculatedDistanceToTarget = val;
        }
        //#endregion
    }

    private static class LimelightKeys {
        public static final String limelight = "limelight";
        public static final String pipeline = "pipeline";
        public static final String tid = "tid";
        public static final String tv = "tv";
        public static final String tx = "tx";
        public static final String ty = "ty";
        public static final String camMode = "camMode";
    }
}
