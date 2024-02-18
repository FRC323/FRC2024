package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.video.KalmanFilter;

/*
 * This class is used to predict the pose of the swerve drivetrain.
 * It uses a Kalman filter to predict the pose of the swerve drivetrain
 *  and should be considered extremely experimental.
 *
 * The correct implementation would be to use an extended or unscented kalman filter for this
 * but those are more complex and this ia first pass.
 *
 */
public class SwervePosePredictor {
  private static final double MAX_SECONDS_PREDICTION = 1.0;
  KalmanFilter kalmanFilter;

  public SwervePosePredictor() {
    this.kalmanFilter = new KalmanFilter(6, 3);
    Mat transitionMatrix = new Mat(6, 6, CvType.CV_32F, new Scalar(0));
    //    This matrix maps the input states <x,y,theta> to the output states <x, x', y, y;, theta,
    // theta'>;
    float[] tM = {
      1, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 1,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1
    };
    transitionMatrix.put(0, 0, tM);
    //    This just maps
    Mat measurementMatrix = new Mat(3, 6, CvType.CV_32F, new Scalar(0));
    measurementMatrix.put(0, 0, 1);
    measurementMatrix.put(1, 1, 1);
    measurementMatrix.put(2, 2, 1);
    kalmanFilter.set_measurementMatrix(measurementMatrix);

    kalmanFilter.set_transitionMatrix(transitionMatrix);
  }

  public Pose2d getPoseAtTime(double secondsInFuture) {

    //    Basically, we make some guesses out into the future
    //    It'll cap out at 50 predictions (1 second/timed robot period)
    long predictionSteps =
        Math.round(Math.max(secondsInFuture, MAX_SECONDS_PREDICTION) / TimedRobot.kDefaultPeriod);
    Mat pred = kalmanFilter.predict();
    for (int i = 1; i < predictionSteps; i++) {
      pred = kalmanFilter.predict(pred);
    }
    return new Pose2d(
        new Translation2d(pred.get(0, 0)[0], pred.get(1, 0)[0]),
        Rotation2d.fromRadians(pred.get(2, 0)[0]));
  }

  public void updatePose(Pose2d pose) {
    Mat measured = new Mat(3, 1, CvType.CV_32FC1);
    measured.put(0, 0, pose.getX());
    measured.put(1, 0, pose.getY());
    measured.put(2, 0, pose.getRotation().getRadians());
    kalmanFilter.correct(measured);
  }
}
