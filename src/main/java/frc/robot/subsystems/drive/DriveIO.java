package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface DriveIO {
  public void setWheelOffsets(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft, SwerveModule rearRight);

  public default void periodic(){};

  Rotation2d getGyroHeading();
  void setGyroHeading(Rotation2d newVal);
}
