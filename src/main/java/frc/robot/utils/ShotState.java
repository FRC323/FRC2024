package frc.robot.utils;

import java.util.Optional;

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

  public ShotState(Rotation2d heading, Rotation2d armAngle, double shooterSpeed) {
    _heading = heading;
    _armAngle = armAngle;
    _shooterSpeed = shooterSpeed;
  }

  public Rotation2d get_armAngle() {
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
    armAngleInterpolation.put(1.1,-0.22);
    armAngleInterpolation.put(2.0, -0.35);
    armAngleInterpolation.put(2.5,-0.49);
    armAngleInterpolation.put(3.0,-0.52);
    armAngleInterpolation.put(3.6,-0.60);
    armAngleInterpolation.put(4.44, -0.61);
    armAngleInterpolation.put(6.0, -0.61);
    return armAngleInterpolation;
  }
}
