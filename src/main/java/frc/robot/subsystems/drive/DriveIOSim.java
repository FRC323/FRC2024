package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

public class DriveIOSim implements DriveIO {

  private SwerveModule frontLeft, frontRight, rearLeft, rearRight;
  private SwerveDriveOdometry odometry;

  public DriveIOSim(
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule rearLeft,
      SwerveModule rearRight) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.rearLeft = rearLeft;
    this.rearRight = rearRight;
    odometry =
        new SwerveDriveOdometry(
            Constants.Swerve.DRIVE_KINEMATICS, Rotation2d.fromDegrees(0.0), getModulePositions());
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      rearLeft.getPosition(),
      rearRight.getPosition()
    };
  }

  @Override
  public void setWheelOffsets(
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule rearLeft,
      SwerveModule rearRight) {
    return;
  }

  @Override
  public Rotation2d getGyroHeading() {
    return new Rotation2d();
  }

  @Override
  public void setGyroHeading(Rotation2d newVal) {}

  public void periodic() {
      odometry.update(getGyroHeading(), getModulePositions());
  }
}
