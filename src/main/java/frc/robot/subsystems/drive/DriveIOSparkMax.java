package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class DriveIOSparkMax implements DriveIO {

  AHRS navx;

  public DriveIOSparkMax() {
    navx = new AHRS(SerialPort.Port.kMXP);
    navx.reset();
  }

  @Override
  public void setWheelOffsets(
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule rearLeft,
      SwerveModule rearRight) {
    double frontLeftOffset = frontLeft.getEncoderAbsPositionRad();
    double frontRightOffset = frontRight.getEncoderAbsPositionRad();
    double rearLeftOffset = rearLeft.getEncoderAbsPositionRad();
    double rearRightOffset = rearRight.getEncoderAbsPositionRad();

    Preferences.setDouble(Constants.Swerve.FRONT_LEFT_OFFSET_KEY, frontLeftOffset);
    Preferences.setDouble(Constants.Swerve.FRONT_RIGHT_OFFSET_KEY, frontRightOffset);
    Preferences.setDouble(Constants.Swerve.REAR_RIGHT_OFFSET_KEY, rearRightOffset);
    Preferences.setDouble(Constants.Swerve.REAR_LEFT_OFFSET_KEY, rearLeftOffset);

    frontLeft.setModuleOffset(frontLeftOffset);
    frontRight.setModuleOffset(frontRightOffset);
    rearLeft.setModuleOffset(rearLeftOffset);
    rearRight.setModuleOffset(rearRightOffset);
  }

  @Override
  public Rotation2d getGyroHeading() {
    return Rotation2d.fromDegrees(navx.getAngle() * (Constants.Swerve.GYRO_REVERSED ? -1 : 1));
  }

  @Override
  public void setGyroHeading(Rotation2d newVal) {
    navx.reset();
    navx.setAngleAdjustment(-newVal.getDegrees());
  }
}
