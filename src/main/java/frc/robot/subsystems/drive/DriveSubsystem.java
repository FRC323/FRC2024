package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.vision.PoseEstimation;
import frc.robot.utils.GeometryUtils;
import frc.robot.utils.ShotState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.Preferences;

public class DriveSubsystem extends SubsystemBase {
  private double targetHeadingDegrees;
  private double previousTargetHeading = 0;
    
  private final PoseEstimation _centerVision = new PoseEstimation(Constants.Vision.CENTER_CAMERA_NAME, Constants.Vision.CENTER_CAMERA_TO_ROBOT);
  private final PoseEstimation _leftVision = new PoseEstimation(Constants.Vision.LEFT_CAMERA_NAME, Constants.Vision.LEFT_CAMERA_TO_ROBOT);
  private final PoseEstimation _rightVision = new PoseEstimation(Constants.Vision.RIGHT_CAMERA_NAME, Constants.Vision.RIGHT_CAMERA_TO_ROBOT);

  private final Field2d _field = new Field2d();

  private ShotState shotState = new ShotState(new Rotation2d(0.0), new Rotation2d(0.0), 0.0); 

  @SuppressWarnings({"FieldCanBeLocal", "FieldMayBeFinal"})
  private double throttleMultiplier = 1.0;

  private ChassisSpeeds lastSetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  AHRS navx;
  private final SwerveModule frontLeft =
      new SwerveModule(
          Constants.Swerve.FRONT_LEFT_DRIVING_CAN_ID,
          Constants.Swerve.FRONT_LEFT_TURNING_CAN_ID,
          Preferences.getDouble(
              Constants.Swerve.FRONT_LEFT_OFFSET_KEY,
              Constants.Swerve.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD),
          Constants.Swerve.FRONT_LEFT_IS_INVERTED);

  private final SwerveModule frontRight =
      new SwerveModule(
          Constants.Swerve.FRONT_RIGHT_DRIVING_CAN_ID,
          Constants.Swerve.FRONT_RIGHT_TURNING_CAN_ID,
          Preferences.getDouble(
              Constants.Swerve.FRONT_RIGHT_OFFSET_KEY,
              Constants.Swerve.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD),
          Constants.Swerve.FRONT_RIGHT_IS_INVERTED);

  private final SwerveModule rearLeft =
      new SwerveModule(
          Constants.Swerve.REAR_LEFT_DRIVING_CAN_ID,
          Constants.Swerve.REAR_LEFT_TURNING_CAN_ID,
          Preferences.getDouble(
              Constants.Swerve.REAR_LEFT_OFFSET_KEY,
              Constants.Swerve.REAR_LEFT_CHASSIS_ANGULAR_OFFSET_RAD),
          Constants.Swerve.REAR_LEFT_IS_INVERTED);

  private final SwerveModule rearRight =
      new SwerveModule(
          Constants.Swerve.REAR_RIGHT_DRIVING_CAN_ID,
          Constants.Swerve.REAR_RIGHT_TURNING_CAN_ID,
          Preferences.getDouble(
              Constants.Swerve.REAR_RIGHT_OFFSET_KEY,
              Constants.Swerve.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD),
          Constants.Swerve.REAR_RIGHT_IS_INVERTED);

  private PIDController rotController = new PIDController(
        Constants.Swerve.ROT_CONTROLLER_KP, 
        Constants.Swerve.ROT_CONTROLLER_KI,
        Constants.Swerve.ROT_CONTROLLER_KD
    );

  SwerveDrivePoseEstimator odometry;

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return odometry;
  }

  public Field2d getField() {
    return _field;
  }

  public void updateSim() {

    if (Constants.Vision.USE_CENTER_CAMERA)
      _centerVision.simulationPeriodic(Constants.Vision.INIT_SIM_POSE);
    if (Constants.Vision.USE_LEFT_CAMERA)
      _leftVision.simulationPeriodic(Constants.Vision.INIT_SIM_POSE);
    if (Constants.Vision.USE_RIGHT_CAMERA)
      _rightVision.simulationPeriodic(Constants.Vision.INIT_SIM_POSE);
  }

  private ChassisSpeeds actualChassisSpeed;

  public DriveSubsystem() {
    navx = new AHRS(SerialPort.Port.kMXP);
    actualChassisSpeed = new ChassisSpeeds(0, 0, 0);
    navx.reset();

    rotController.enableContinuousInput(-Math.PI, Math.PI);

    odometry = new SwerveDrivePoseEstimator(
                Constants.Swerve.DRIVE_KINEMATICS,
                Rotation2d.fromDegrees(this.getGyroYaw()),
                this.getModulePositions(),
                new Pose2d());

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getChassisSpeed,
        this::setPathFollowerSpeeds,
        Constants.PathFollowing.holonomicPathFollowerConfig,
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

    if (Constants.Vision.USE_CENTER_CAMERA)
      updatePoseEstimation(_centerVision);
    if (Constants.Vision.USE_LEFT_CAMERA)
      updatePoseEstimation(_leftVision);
    if (Constants.Vision.USE_RIGHT_CAMERA)
      updatePoseEstimation(_rightVision);

    //compute shot state for auto aiming
    computeShotState();

    odometry.update(Rotation2d.fromDegrees(getGyroYaw()), getModulePositions());
    actualChassisSpeed = Constants.Swerve.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());

    if (Constants.LOGGING.SHOW_2D_FIELD)
      _field.setRobotPose(getPose());
  }

  private void updatePoseEstimation(PoseEstimation poseEstimation) {
    var estimatedPoseOpt = poseEstimation.getEstimatedGlobalPose();
    if (PoseEstimation.checkIfValidPose(estimatedPoseOpt)) {
      var estimatedPose = estimatedPoseOpt.get();
      var estimatedPose2d = estimatedPose.estimatedPose.toPose2d();
      getPoseEstimator().addVisionMeasurement(estimatedPose2d, estimatedPose.timestampSeconds, poseEstimation.getEstimationStdDevs(estimatedPose2d));
    }
  }

  private void computeShotState(){
    //Shot Target
    if(DriverStation.getAlliance().isEmpty()){
        return;
    } 
    var shotTarget = DriverStation.getAlliance().get() == Alliance.Red ? Vision.RED_SHOT_TARGET : Vision.BLUE_SHOT_TARGET;

    //Robot Velocity
    var robotVelocity = this.getChassisSpeed();

    //dt (Todo: find actual dt)
    var dt = DriverStation.isAutonomous() ? 0.0 : 0.5;

    this.shotState =  ShotState.computedFromPose(
        shotTarget,
        // rangeToTarget,
        getPose(),
        robotVelocity,
        dt
      );
  }

  public ShotState getShotState() {
    return this.shotState;
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

  //public boolean updateOdometry() {
    // System.out.println("updating odometry");
    // if (canSeeTargets()) {
    //     resetYawToAngle(getEstimatedRobotPose().estimatedPose.getRotation().toRotation2d().rotateBy(new Rotation2d(Math.PI)).getDegrees());
    //    // resetYawToAngle(getPose().getRotation().rotateBy(new Rotation2d(Math.PI)).getDegrees());
    //     resetOdometry(getPose());
    //     System.out.println("Updated Odometry");
    //     return true;
    // } else {
    //   return false;
    // }
  //}

  public boolean canSeeTargets() {
    if (Constants.Vision.USE_CENTER_CAMERA && _centerVision.canSeeTargets())
      return true;
    if (Constants.Vision.USE_LEFT_CAMERA && _leftVision.canSeeTargets())
      return true;
    if (Constants.Vision.USE_RIGHT_CAMERA && _rightVision.canSeeTargets())
      return true;
    return false;
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
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

  public void resetPose(Pose2d pose) {
    setGyroYaw(getPose().getRotation().getDegrees());
    odometry.resetPosition(Rotation2d.fromDegrees(getGyroYaw()), getModulePositions(), pose);
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

  private MedianFilter filter = new MedianFilter(5);

  public void driveWithHeading(double xSpeed, double ySpeed, Rotation2d targetHeadingRads, boolean fieldRelative){
    var delta = filter.calculate((targetHeadingRads.getRadians() - previousTargetHeading));
    
    rotController.setSetpoint(targetHeadingRads.getRadians() + delta*20);

    //+ 5 * actualChassisSpeed.omegaRadiansPerSecond/Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS

    double rotControllerValue = rotController.calculate(
        this.getPose().getRotation().getRadians() % (Math.PI * 2)
      );

    drive(
      xSpeed,
      ySpeed,
      (rotControllerValue) / Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS,
      fieldRelative
    );

    previousTargetHeading = targetHeadingRads.getRadians();
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

    builder.addDoubleProperty("Actual Velocity Omega", () -> actualChassisSpeed.omegaRadiansPerSecond, null);

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

    builder.addDoubleProperty(
        "Front Left Speed", () -> frontLeft.getModuleVelocity(), null); 
    builder.addDoubleProperty(
        "Front Right Speed", () -> Math.abs(frontRight.getModuleVelocity()), null);
    builder.addDoubleProperty(
        "Rear Left Speed", () -> Math.abs(rearLeft.getModuleVelocity()), null);
    builder.addDoubleProperty(
        "Rear Right Speed", () -> Math.abs(rearRight.getModuleVelocity()), null);


    builder.addDoubleProperty(
        "Front Left Desired Speed", () -> frontLeft.getDesiredModuleVelocity(), null); 
    builder.addDoubleProperty(
        "Front Left Acceleration", () -> frontLeft.getModuleAcceleration(), null);
    builder.addDoubleProperty(
        "Front Left Desired Acceleration", () -> frontLeft.getDesiredModuleAcceleration(), null);
    builder.addDoubleProperty(
        "Front Left Jerk", () -> Math.abs(frontLeft.getModuleJerk()), null);
    builder.addDoubleProperty(
        "Front Left: Jerk per Current", () -> frontLeft.getModuleJerktoCurrent(), null);

    if (Constants.Vision.USE_RIGHT_CAMERA)
      builder.addIntegerArrayProperty("PV Right View", () -> _rightVision.targetsInView().stream().mapToLong(i -> i.getFiducialId()).toArray(), null);
    if (Constants.Vision.USE_LEFT_CAMERA)
      builder.addIntegerArrayProperty("PV Left View", () -> _leftVision.targetsInView().stream().mapToLong(i -> i.getFiducialId()).toArray(), null);
    if (Constants.Vision.USE_CENTER_CAMERA)
      builder.addIntegerArrayProperty("PV Center View", () -> _centerVision.targetsInView().stream().mapToLong(i -> i.getFiducialId()).toArray(), null);

    builder.addDoubleProperty(
        "Front Left Current", () -> Math.abs(frontLeft.getCurrent()), null);
    builder.addDoubleProperty(
        "Front Right Current", () -> Math.abs(frontRight.getCurrent()), null);
    builder.addDoubleProperty(
        "Rear Left Current", () -> Math.abs(rearLeft.getCurrent()), null);
    builder.addDoubleProperty(
        "Rear Right Current", () -> Math.abs(rearRight.getCurrent()), null);

    builder.addDoubleProperty("Rear Right Velocity", rearRight::getModuleVelocity, null);
    builder.addDoubleProperty("RR Current",rearRight::getCurrent, null);
    builder.addDoubleProperty("Rot Error", () -> rotController.getPositionError() , null);
    builder.addDoubleProperty("Rot Target", () -> rotController.getSetpoint() , null);
  }
}
