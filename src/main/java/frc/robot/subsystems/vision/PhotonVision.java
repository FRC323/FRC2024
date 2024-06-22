package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonVision {
    private PhotonCamera _camera;
    private PhotonPoseEstimator _estimator;
    private Optional<EstimatedRobotPose> _estimatedPose;
    private String _cameraName;
    private Transform3d _cameraToRobotTransform;
    private AprilTagFieldLayout AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public String getCameraName() {
        return _cameraName;
    }

    public Transform3d getTransform() {
        return _cameraToRobotTransform;
    }

    public PhotonPipelineResult getLatestResult() {
        return _camera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return _estimatedPose;
    }

    public boolean hasTargets() {
        var result = getLatestResult();
        return result.hasTargets();
    }

    public PhotonVision(String cameraName, Transform3d translation) {
        _camera = new PhotonCamera(cameraName);
        _cameraName = cameraName;
        _cameraToRobotTransform = translation;
        _estimator = new PhotonPoseEstimator(AprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, _camera, _cameraToRobotTransform);
        _estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void updateEstimator() {
        _estimatedPose = _estimator.update();
    }
}
