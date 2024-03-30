package frc.robot.subsystems.drive;

import com.fasterxml.jackson.core.StreamReadConstraints.Builder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PathFollowing;
import frc.robot.utils.GeometryUtils;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class DriveSubsystem extends SubsystemBase {
  private double targetHeadingDegrees;

  @SuppressWarnings({"FieldCanBeLocal", "FieldMayBeFinal"})
  private double throttleMultiplier = 1.0;

  private ChassisSpeeds lastSetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  AHRS navx;
  public final SwerveModule frontLeft =
      new SwerveModule(
          Constants.Swerve.FRONT_LEFT_DRIVING_CAN_ID,
          Constants.Swerve.FRONT_LEFT_TURNING_CAN_ID,
          Preferences.getDouble(
              Constants.Swerve.FRONT_LEFT_OFFSET_KEY,
              Constants.Swerve.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD),
          Constants.Swerve.FRONT_LEFT_IS_INVERTED);

  public final SwerveModule frontRight =
      new SwerveModule(
          Constants.Swerve.FRONT_RIGHT_DRIVING_CAN_ID,
          Constants.Swerve.FRONT_RIGHT_TURNING_CAN_ID,
          Preferences.getDouble(
              Constants.Swerve.FRONT_RIGHT_OFFSET_KEY,
              Constants.Swerve.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD),
          Constants.Swerve.FRONT_RIGHT_IS_INVERTED);

  public final SwerveModule rearLeft =
      new SwerveModule(
          Constants.Swerve.REAR_LEFT_DRIVING_CAN_ID,
          Constants.Swerve.REAR_LEFT_TURNING_CAN_ID,
          Preferences.getDouble(
              Constants.Swerve.REAR_LEFT_OFFSET_KEY,
              Constants.Swerve.REAR_LEFT_CHASSIS_ANGULAR_OFFSET_RAD),
          Constants.Swerve.REAR_LEFT_IS_INVERTED);

  public final SwerveModule rearRight =
      new SwerveModule(
          Constants.Swerve.REAR_RIGHT_DRIVING_CAN_ID,
          Constants.Swerve.REAR_RIGHT_TURNING_CAN_ID,
          Preferences.getDouble(
              Constants.Swerve.REAR_RIGHT_OFFSET_KEY,
              Constants.Swerve.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD),
          Constants.Swerve.REAR_RIGHT_IS_INVERTED);

  private ChassisSpeeds lastSetChassisSpeed = new ChassisSpeeds(0, 0, 0);
  private Optional<Pose2d> targetPose = Optional.empty();

  private PIDController rotController = new PIDController(
        6.0, 
        0.1,
        0.1
    );

  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          Constants.Swerve.DRIVE_KINEMATICS, Rotation2d.fromDegrees(0.0), getModulePositions());
  private ChassisSpeeds actualChassisSpeed;

  public DriveSubsystem() {
    navx = new AHRS(SerialPort.Port.kMXP);
    actualChassisSpeed = new ChassisSpeeds(0, 0, 0);
    navx.reset();
    // TODO: Make this in a different initilztion place

    rotController.enableContinuousInput(-Math.PI, Math.PI);
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeed,
        this::setPathFollowerSpeeds,
        Constants.PathFollowing.holonomicPathFollowerConfig,
        // this::mirrorForRedAlliance,
        ()->false,
        this);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    frontLeft.periodic();
    frontRight.periodic();
    rearLeft.periodic();
    rearRight.periodic();
    odometry.update(Rotation2d.fromDegrees(getGyroYaw()), getModulePositions());
    actualChassisSpeed = Constants.Swerve.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()
    };
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

  public ChassisSpeeds getChassisSpeed() {
    return actualChassisSpeed;
  }

  // Useful for figuring out if we should go to X pose
  public ChassisSpeeds getLastSetChassisSpeed() {
    return lastSetChassisSpeeds;
  }

  //  Useful for resetting the pose off of april tag
  public void resetOdometry(Pose2d pose) {
    // Just update the translation, not the yaw
    Pose2d resetPose = new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(getGyroYaw()));
    odometry.resetPosition(Rotation2d.fromDegrees(getGyroYaw()), getModulePositions(), resetPose);
  }

  public Pose2d getRobotPose2d(){
    return odometry.getPoseMeters();
  }

  public double getGyroYaw() {
    // TODO: Handle Gyro Reverse
    return navx.getAngle() * (Constants.Swerve.GYRO_REVERSED ? -1 : 1);
  }
  
  public void setGyroYaw(double yawDeg) {
    // I'm not 100% sure on this to be honest
    // Reset it to 0, then add an offset negative what you want.
    // TODO: Handle Gyro Reverse
    navx.reset();
    navx.setAngleAdjustment(yawDeg);
    System.out.println("Nax Offset: " + (yawDeg));
  }

  public void resetYawToAngle(double yawDeg) {
    System.out.println("Gyro Reset To: "+ yawDeg);
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

  public void setWheelOffsets() {
    double frontLeftOffset = frontLeft.getEncoderAbsPositionRad();
    double frontRightOffset = frontRight.getEncoderAbsPositionRad();
    double rearLeftOffset = rearLeft.getEncoderAbsPositionRad();
    double rearRightOffset = rearRight.getEncoderAbsPositionRad();

    Preferences.setDouble(Constants.Swerve.FRONT_LEFT_OFFSET_KEY, frontLeftOffset);
    Preferences.setDouble(Constants.Swerve.FRONT_RIGHT_OFFSET_KEY, frontRightOffset);
    Preferences.setDouble(Constants.Swerve.REAR_LEFT_OFFSET_KEY, rearLeftOffset);
    Preferences.setDouble(Constants.Swerve.REAR_RIGHT_OFFSET_KEY, rearRightOffset);

    frontLeft.setModuleOffset(frontLeftOffset);
    frontRight.setModuleOffset(frontRightOffset);
    rearLeft.setModuleOffset(rearLeftOffset);
    rearRight.setModuleOffset(rearRightOffset);
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

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Desired speed of the robot in the x direction (forward), [-1,1].
   * @param ySpeed Desired speed of the robot in the y direction (sideways), [-1,1].
   * @param rot Desired angular rate of the robot, [-1,1].
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // Adjust input based on max speed
    xSpeed *= Constants.Swerve.MAX_SPEED_METERS_PER_SECOND;
    ySpeed *= Constants.Swerve.MAX_SPEED_METERS_PER_SECOND;
    rot *= Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS;

    xSpeed *= throttleMultiplier;
    ySpeed *= throttleMultiplier;
    rot *= throttleMultiplier;

    ChassisSpeeds desiredChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getGyroYaw()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
    //    This is the last commanded chassis speed not the last actual chassis speed
    this.lastSetChassisSpeeds = desiredChassisSpeeds;

    var swerveModuleStates =
        Constants.Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveWithHeading(double xSpeed, double ySpeed, Rotation2d targetHeadingRads,boolean fieldRelative){
    rotController.setSetpoint(targetHeadingRads.getRadians());
    
    drive(
      xSpeed,
      ySpeed,
      rotController.calculate(
        this.getPose().getRotation().getRadians() % (Math.PI * 2)
      ) / Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS,
      fieldRelative
    );
  }

  public void turnToHeading(Rotation2d targeRotation2d){
    rotController.setSetpoint(targeRotation2d.getRadians());

    drive(
      0.0,
      0.0,
      rotController.calculate(
        this.getPose().getRotation().getRadians() % (Math.PI * 2)
      ) / Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS,
      false
    );
  }

  public void setPathFollowerSpeeds(ChassisSpeeds speeds) {
    setChassisSpeed(speeds);
  }

  public void setChassisSpeed(ChassisSpeeds speed) {
    double maxSpeed = Constants.Swerve.MAX_SPEED_METERS_PER_SECOND;
    double maxRotationalSpeed =
        Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS;
    drive(
        speed.vxMetersPerSecond / maxSpeed,
        speed.vyMetersPerSecond / maxSpeed,
        speed.omegaRadiansPerSecond / maxRotationalSpeed,
        false);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  public boolean atHeading(){
    return this.rotController.atSetpoint();
  }

  private boolean mirrorForRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    addChild("Front Right", frontRight);
    addChild("Front Left", frontLeft);
    addChild("Rear Right", rearRight);
    addChild("Rear Left", rearLeft);

    builder.addDoubleProperty(
        "Target Velocity X", () -> this.lastSetChassisSpeeds.vxMetersPerSecond, null);
    builder.addDoubleProperty(
            "Target Velocity Y", () -> this.lastSetChassisSpeeds.vyMetersPerSecond, null);
    builder.addDoubleProperty("Actual Velocity X", () -> actualChassisSpeed.vxMetersPerSecond, null);
    builder.addDoubleProperty("Actual Velocity Y", () -> actualChassisSpeed.vyMetersPerSecond, null);

    builder.addDoubleProperty("Gyro Yaw (deg)", this::getGyroYaw, null);
    builder.addDoubleProperty("Odometry X (m)", () -> getPose().getX(), null);
    builder.addDoubleProperty("Odometry Y (m)", () -> getPose().getY(), null);
    builder.addDoubleProperty(
        "Odometry Yaw (deg)", () -> (getPose().getRotation().getDegrees()%360.0), null);
    builder.addDoubleProperty(
        "Front Left Abs Encoder (rad)", frontLeft::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Front Right Abs Encoder (rad)", frontRight::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Rear Left Abs Encoder (rad)", rearLeft::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Rear Right Abs Encoder (rad)", rearRight::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Front Left Module Pos (rad)", () -> frontLeft.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Right Module Pos (rad)", () -> frontRight.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear Left Module Pos (rad)", () -> rearLeft.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear Right Module Pos (rad)", () -> rearRight.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Left Distance (m)", () -> frontLeft.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Front Right Distance (m)", () -> frontRight.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Rear Left Distance (m)", () -> rearLeft.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Rear Right Distance (m)", () -> rearRight.getPosition().distanceMeters, null);
    builder.addDoubleProperty("Rear Right Velocity", rearRight::getModuleVelocity, null);
    builder.addDoubleProperty("RR Current",rearRight::getCurrent, null);
    builder.addDoubleProperty("Rot Error", () -> rotController.getPositionError() , null);
  }
}
