package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;

import java.util.Optional;

public interface DriveIO extends Sendable {
    public static class DriveIOInputs {
        public  Pose2d currentPose;
        public Optional<Pose2d> targetPose;
        public SwerveModulePosition[] modulePositions;
        public ChassisSpeeds lastChassisSpeed;
        public ChassisSpeeds currentChassisSpeed;
    }


    public void drive(double xVel, double yVel, double rotVel, boolean isFieldRelative);
    public void setWheelOffsets();
}
