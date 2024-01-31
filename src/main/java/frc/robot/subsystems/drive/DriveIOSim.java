package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public class DriveIOSim implements DriveIO {

    @Override
    public void setWheelOffsets(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft, SwerveModule rearRight) {
        return;
    }

    @Override
    public Rotation2d getGyroHeading() {
    return new Rotation2d();
    }

    @Override
    public void setGyroHeading(Rotation2d newVal) {

    }
}
