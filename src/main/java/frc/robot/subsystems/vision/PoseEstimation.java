package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Functions;

public class PoseEstimation {
    private PhotonCamera _camera;
    private PhotonPoseEstimator _estimator;
    private double _lastEstTimestamp = 0;

    //sim
    private PhotonCameraSim _cameraSim;
    private VisionSystemSim _visionSim;

    public PoseEstimation(String cameraName, Transform3d robotToCamera) {
        _camera = new PhotonCamera(cameraName);
        _estimator = new PhotonPoseEstimator(Constants.Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, _camera, robotToCamera);

        _estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            _visionSim = new VisionSystemSim("RearCamera");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            _visionSim.addAprilTags(Constants.Vision.kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70));
            cameraProp.setCalibError(0.25, 0.08);
            cameraProp.setFPS(20);
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            _cameraSim = new PhotonCameraSim(_camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            _visionSim.addCamera(_cameraSim, robotToCamera);

            _cameraSim.enableDrawWireframe(true);

            SmartDashboard.putData("VisionSimulation", _visionSim.getDebugField());
        }
    }

    public PhotonPipelineResult getLatestResult() {
        return _camera.getLatestResult();
    }

    public static boolean checkIfValidPose(Optional<EstimatedRobotPose> estimatedPose) {
        if (!estimatedPose.isPresent())
            return false;

        var pose = estimatedPose.get().estimatedPose.toPose2d();

        var isWithinField = Functions.isInRange(pose.getY(), -5, Constants.Vision.kTagLayout.getFieldWidth() + 5)
            && Functions.isInRange(pose.getX(), -5, Constants.Vision.kTagLayout.getFieldLength() + 5);

        return isWithinField;
    }

    public boolean canSeeTargets() {
        return _camera.getLatestResult().hasTargets();
    }

    public List<PhotonTrackedTarget> targetsInView() {
        return _camera.getLatestResult().getTargets();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = _estimator.update();
        double latestTimestamp = _camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - _lastEstTimestamp) > 1e-5;
        if (newResult) _lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = _camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = _estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public void simulationPeriodic(Pose2d robotSimPose) {
        _visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) _visionSim.resetRobotPose(pose);
    }
}
