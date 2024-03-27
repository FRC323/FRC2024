package frc.robot.utils;

import java.util.Optional;

import javax.sound.sampled.Line;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ShotState {
  private final Rotation2d _heading;
  private final Rotation2d _armAngle;
  private final double _shooterSpeed;
  private static final InterpolatingDoubleTreeMap armAngleInterpolation =
      initializeInterpolator();

    private SlewRateLimiter headingLimiter = new SlewRateLimiter(Math.PI);
    private SlewRateLimiter armAngleLimiter = new SlewRateLimiter(4 * Math.PI);
    // private LinearFilter shooterSpeedLimiter = new SlewRateLimiter();

  public ShotState(Rotation2d heading, Rotation2d armAngle, double shooterSpeed) {
    _heading = heading;
    _armAngle = armAngle;
    _shooterSpeed = shooterSpeed;
  }

  public Rotation2d get_armAngle() {
    // return Rotation2d.fromRadians(armAngleLimiter.calculate(_armAngle.getRadians()));
    return _armAngle;
  }

  public Rotation2d get_heading() {
    return _heading;
  }

  public double get_shooterSpeed() {
    return _shooterSpeed;
  }



  public static ShotState computedFromPose(
      Translation2d shotTarget,
      // double rangeToTarget,
      Pose2d robotPose,
      ChassisSpeeds robotVel,
      double dt) {
    // This computes the distance we will travel (in meters) in the prediction time
    ChassisSpeeds robotPoseDelta = robotVel.times(dt);
    Translation2d offset =
        new Translation2d(robotPoseDelta.vxMetersPerSecond, robotPoseDelta.vyMetersPerSecond);

    // Now we need to offset the target point by that
    // Note, minus is correct, we want to lead the shot so we figure out where the robot will be in
    // the future and place the target accordingly (I think)
    Translation2d futureShotTarget = shotTarget.plus(offset);
    // Then we extract the robot translation
    Translation2d robotPosition = robotPose.getTranslation();
    // Then build the triangle
    Translation2d triangle = futureShotTarget.minus(robotPosition);
    // Then use the hypotenuse to compute the angle
    // The angle is the arctan of the opposite over adjacent
    Rotation2d chassisAngle = new Rotation2d(triangle.getX(), triangle.getY());
    chassisAngle = chassisAngle.rotateBy(Rotation2d.fromRadians(Math.PI));

    var rangeToTarget = futureShotTarget.getDistance(robotPosition);
    // Now build a new ShotState object
    return new ShotState(
        chassisAngle,
        Rotation2d.fromRadians(armAngleInterpolation.get(rangeToTarget)),
        Constants.Shooter.SHOOTER_SPEED);
  }

  private static InterpolatingDoubleTreeMap initializeInterpolator() {
    var armAngleInterpolation = new InterpolatingDoubleTreeMap();
    armAngleInterpolation.put(0.0, 0.0);
    armAngleInterpolation.put(0.75, -0.12);
    armAngleInterpolation.put(0.9,-0.25);
    armAngleInterpolation.put(2.0, -0.44);
    armAngleInterpolation.put(2.5,-0.54);
    armAngleInterpolation.put(3.0,-0.64);
    armAngleInterpolation.put(3.5,-0.67);
    armAngleInterpolation.put(4.0, -0.71);
    armAngleInterpolation.put(4.5, -0.75);
    armAngleInterpolation.put(5.0, -0.76);
    return armAngleInterpolation;
  }
}
