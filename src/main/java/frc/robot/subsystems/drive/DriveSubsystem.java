package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.GeometryUtils;

import java.util.Optional;

public class DriveSubsystem extends SubsystemBase {
  private double targetHeadingDegrees;
  AHRS navx;
  public final SwerveModule frontLeft =
      new SwerveModule(
          Constants.Swerve.FRONT_LEFT_DRIVING_CAN_ID,
          Constants.Swerve.FRONT_LEFT_TURNING_CAN_ID,
          Constants.Swerve.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule frontRight =
      new SwerveModule(
          Constants.Swerve.FRONT_RIGHT_DRIVING_CAN_ID,
          Constants.Swerve.FRONT_RIGHT_TURNING_CAN_ID,
          Constants.Swerve.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule rearLeft =
      new SwerveModule(
          Constants.Swerve.REAR_LEFT_DRIVING_CAN_ID,
          Constants.Swerve.REAR_LEFT_TURNING_CAN_ID,
          Constants.Swerve.REAR_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule rearRight =
      new SwerveModule(
          Constants.Swerve.REAR_RIGHT_DRIVING_CAN_ID,
          Constants.Swerve.REAR_RIGHT_TURNING_CAN_ID,
          Constants.Swerve.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  private ChassisSpeeds lastSetChassisSpeed = new ChassisSpeeds(0, 0, 0);
  private Optional<Pose2d> targetPose = Optional.empty();

  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          Constants.Swerve.DRIVE_KINEMATICS, Rotation2d.fromDegrees(0.0), getModulePositions());

  public DriveSubsystem() {
    navx = new AHRS(SerialPort.Port.kMXP);
    navx.reset();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    frontLeft.periodic();
    frontRight.periodic();
    rearLeft.periodic();
    rearRight.periodic();
    odometry.update(Rotation2d.fromDegrees(getGyroYaw()), getModulePositions());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
    };
  }

  public void burnFlashSparks() {
    frontLeft.burnFlashSparks();
    frontRight.burnFlashSparks();
    rearLeft.burnFlashSparks();
    rearRight.burnFlashSparks();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

//  Useful for resetting the pose off of april tag
  public void resetOdometry(Pose2d pose) {
    // Just update the translation, not the yaw
    Pose2d resetPose = new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(getGyroYaw()));
    odometry.resetPosition(Rotation2d.fromDegrees(getGyroYaw()), getModulePositions(), resetPose);
  }

  public double getGyroYaw() {
    navx.getAngle();
  }

  public void setGyroYaw(double yawDeg) {
    // I'm not 100% sure on this to be honest
    // Reset it to 0, then add an offset negative what you want.
    navx.reset();
    navx.setAngleAdjustment(-yawDeg);
  }

  public void resetYawToAngle(double yawDeg) {
    double curYawDeg = getGyroYaw();
    double offsetToTargetDeg = targetHeadingDegrees - curYawDeg;
    setGyroYaw(yawDeg);
    Pose2d curPose = getPose();
    Pose2d resetPose = new Pose2d(curPose.getTranslation(), Rotation2d.fromDegrees(yawDeg));
    odometry.resetPosition(Rotation2d.fromDegrees(yawDeg), getModulePositions(), resetPose);
    targetHeadingDegrees = yawDeg + offsetToTargetDeg;
  }

  public void resetYaw() {
    resetYawToAngle(0.0);
  }

  /**
   * Correction for swerve second order dynamics issue. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose =
            new Pose2d(
                    originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                    originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                    Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                    twistForPose.dx / LOOP_TIME_S,
                    twistForPose.dy / LOOP_TIME_S,
                    twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }
}


