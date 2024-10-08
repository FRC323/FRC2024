package frc.robot.subsystems.drive;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.utils.AbsoluteEncoderChecker;
import frc.robot.utils.SparkMaxUtils;

import static frc.robot.utils.SparkMaxUtils.check;

public class SwerveModule implements Sendable {

  private final CANSparkMax turningSpark;
  private final CANSparkMax drivingSpark;
  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;
  private AbsoluteEncoderChecker turningAbsoluteEncoderChecker = new AbsoluteEncoderChecker();
  private final SparkPIDController drivingPIDController;
  private final SparkPIDController turningPIDController;
  //    Radians offset for the module
  private double moduleOffset;
  private final boolean isInverted;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d(0));

  private double drivingEncoderPrevVelocity = 0;
  private double moduleAcceleration = 0;

  private double drivingEncoderPrevAcceleration = 0;
  private double moduleJerk = 0;

  private double lastNonSlippingWheelVelocity = 0;
  private double count = 0;

  private double desiredModuleAcceleration = 0;
  private double desiredModuleVelocity = 0;

  private Rotation2d previousMoudleAngle = Rotation2d.fromRadians(0);
  private double lastNonSlippingWheelAcceleration = 0;

  public SwerveModule(int drivingCanId, int turningCanId, double moduleOffset,boolean isInverted) {
    drivingSpark = new CANSparkMax(drivingCanId, MotorType.kBrushless);
    turningSpark = new CANSparkMax(turningCanId, MotorType.kBrushless);
    this.moduleOffset = moduleOffset;
    this.isInverted = isInverted;

    SparkMaxUtils.initWithRetry(this::initDriveSpark, Constants.SPARK_INIT_RETRY_ATTEMPTS);
    SparkMaxUtils.initWithRetry(this::initTurnSpark, Constants.SPARK_INIT_RETRY_ATTEMPTS);

    drivingEncoder = drivingSpark.getEncoder();
    drivingPIDController = drivingSpark.getPIDController();
    turningEncoder = turningSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    turningPIDController = turningSpark.getPIDController();

    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0.0);
    
  }

  private boolean initTurnSpark() {
    int errors = 0;
    errors += check(turningSpark.restoreFactoryDefaults());
    AbsoluteEncoder turningEncoderTmp =
        turningSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    SparkPIDController turningPidTmp = turningSpark.getPIDController();
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setRadsFromGearRatio(turningEncoderTmp, 1.0));
    errors += check(turningPidTmp.setFeedbackDevice(turningEncoderTmp));
    errors +=
        check(turningEncoderTmp.setInverted(Constants.Swerve.Module.TURNING_ENCODER_INVERTED));
    errors += check(turningPidTmp.setPositionPIDWrappingEnabled(true));
    errors +=
        check(
            turningPidTmp.setPositionPIDWrappingMinInput(
                Constants.Swerve.Module.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS));
    errors +=
        check(
            turningPidTmp.setPositionPIDWrappingMaxInput(
                Constants.Swerve.Module.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS));
    errors += check(turningPidTmp.setP(Constants.Swerve.Module.TURNING_K_P));
    errors += check(turningPidTmp.setI(Constants.Swerve.Module.TURNING_K_I));
    errors += check(turningPidTmp.setD(Constants.Swerve.Module.TURNING_K_D));
    errors += check(turningPidTmp.setFF(Constants.Swerve.Module.TURNING_K_FF));
    errors +=
        check(
            turningPidTmp.setOutputRange(
                Constants.Swerve.Module.TURNING_MIN_OUTPUT,
                Constants.Swerve.Module.TURNING_MAX_OUTPUT));
    errors += check(turningSpark.setIdleMode(Constants.Swerve.Module.TURNING_MOTOR_IDLE_MODE));
    errors +=
        check(
            turningSpark.setSmartCurrentLimit(
                Constants.Swerve.Module.TURNING_MOTOR_CURRENT_LIMIT_AMPS));
    return errors == 0;
  }

  private boolean initDriveSpark() {
    int errors = 0;
    errors += check(drivingSpark.restoreFactoryDefaults());
    RelativeEncoder drivingEncoderTmp = drivingSpark.getEncoder();
    SparkPIDController drivingPidTmp = drivingSpark.getPIDController();
    errors += check(drivingPidTmp.setFeedbackDevice(drivingEncoderTmp));
//    errors +=
//        check(drivingEncoderTmp.setInverted(Constants.Swerve.Module.DRIVING_ENCODER_INVERTED));
    drivingSpark.setInverted(Constants.Swerve.Module.DRIVING_ENCODER_INVERTED);
    errors += check(drivingPidTmp.setPositionPIDWrappingEnabled(false));
    errors += check(drivingPidTmp.setP(Constants.Swerve.Module.DRIVING_K_P));
    errors += check(drivingPidTmp.setI(Constants.Swerve.Module.DRIVING_K_I));
    errors += check(drivingPidTmp.setD(Constants.Swerve.Module.DRIVING_K_D));
    errors += check(drivingPidTmp.setFF(Constants.Swerve.Module.DRIVING_K_FF));
    errors +=
        check(
            drivingPidTmp.setOutputRange(
                Constants.Swerve.Module.DRIVING_MIN_OUTPUT,
                Constants.Swerve.Module.DRIVING_MAX_OUTPUT));
    errors +=
        SparkMaxUtils.check(
            drivingEncoderTmp.setPositionConversionFactor(
                Constants.Swerve.Module.DRIVING_ENCODER_POSITION_FACTOR_METERS));
    errors +=
        SparkMaxUtils.check(
            drivingEncoderTmp.setVelocityConversionFactor(
                Constants.Swerve.Module.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND));

    errors += check(drivingSpark.setIdleMode(Constants.Swerve.Module.DRIVING_MOTOR_IDLE_MODE));
    errors +=
        check(
            drivingSpark.setSmartCurrentLimit(
                Constants.Swerve.Module.DRIVING_MOTOR_CURRENT_LIMIT_AMPS));
        drivingSpark.setInverted(isInverted);
        drivingSpark.setIdleMode(IdleMode.kBrake);

                return errors == 0;
  }

  //    Writes config to flash so it will persist through power loss
  public void burnFlashSparks() {
    Timer.delay(0.005);
    drivingSpark.burnFlash();
    Timer.delay(0.005);
    turningSpark.burnFlash();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        drivingEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition() - moduleOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        drivingEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition() - moduleOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    updateModuleKinematics();
    
    // count is a filter to deal with jittery jerk values
    if (isModuleJerkWithinThreshold()) {
      count++;
    } else {
      count = 0;
    }
    count = 2;
    // check if count is above 1 as simple filter
    if (count > 1) {
      // In this case, module acts like normal
      applyDesiredState(correctedDesiredState, desiredState);
    } else {
      // In this case, module locks to angle and acceleration
      applyCorrectedState(correctedDesiredState, desiredState);
    }

    SwerveModuleState optimizedState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));
    this.desiredState = optimizedState;

    drivingPIDController.setReference(optimizedState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
    turningPIDController.setReference(optimizedState.angle.getRadians(), CANSparkBase.ControlType.kPosition);
  }
    
  private void updateModuleKinematics() {
    moduleAcceleration = (getModuleVelocity() - drivingEncoderPrevVelocity) * 50;
    drivingEncoderPrevVelocity = getModuleVelocity();
    moduleJerk = (moduleAcceleration - drivingEncoderPrevAcceleration) * 50;
    drivingEncoderPrevAcceleration = moduleAcceleration;
  } 
    
  private boolean isModuleJerkWithinThreshold() {
    return getModuleJerk() < Constants.Swerve.Module.JERK_THRESHOLD;
  }
    
  private void applyDesiredState(SwerveModuleState correctedState, SwerveModuleState desiredState) { 
    correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond; 
    correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(moduleOffset)); 
    lastNonSlippingWheelAcceleration = moduleAcceleration; 
    previousMoudleAngle = desiredState.angle.plus(Rotation2d.fromRadians(moduleOffset)); 
  } 
    
  private void applyCorrectedState(SwerveModuleState correctedState, SwerveModuleState desiredState) {
    //Module acceleration is locked here
    correctedState.speedMetersPerSecond = getModuleVelocity() + lastNonSlippingWheelAcceleration / 50;
    
    if (DriverStation.isAutonomous()) {
      // Disables module angle locking for traction control during auto. Otherwise auto becomes all screwy. 
      // The module anlge locking only matter for extremely sharp changes in desired direction. Which isn't a problem with pre-programmed paths.
      correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(moduleOffset));
    } else {
      // Module angle is locked here
      correctedState.angle = previousMoudleAngle;
    }
  }

  public double getEncoderAbsPositionRad() {
    return turningEncoder.getPosition();
  }

  public double getDesiredModuleVelocity () {
    return desiredModuleVelocity;
  }

  public double getModuleVelocity(){
    return drivingEncoder.getVelocity();
  }

  public double getModuleAcceleration(){
    return moduleAcceleration;
  }

  public double getDesiredModuleAcceleration(){
    return desiredModuleAcceleration;
  }

  public double getModuleJerk(){
    return moduleJerk;
  }

  public double getLastNonSlippingWheelVelocity(){
    return lastNonSlippingWheelVelocity;
  }

  public double getModuleJerktoCurrent(){
    return Math.min(1000, Math.abs(getModuleJerk()) / (getCurrent()+2)); // +1 is for divide by zero error. Also, values shouldn't be too big at low speeds (Where current is 0)
  }

  public double getCurrent(){
    return drivingSpark.getOutputCurrent();
  }

  public void periodic() {
    turningAbsoluteEncoderChecker.addReading(turningEncoder.getPosition());
  }

  public void setModuleOffset(double offset){
    this.moduleOffset = offset;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Driving kP", drivingPIDController::getP, drivingPIDController::setP);
    builder.addDoubleProperty("Driving kI", drivingPIDController::getI, drivingPIDController::setI);
    builder.addDoubleProperty("Driving kD", drivingPIDController::getD, drivingPIDController::setD);
    builder.addDoubleProperty(
        "Driving kFF", drivingPIDController::getFF, drivingPIDController::setFF);
    builder.addDoubleProperty("Turning kP", turningPIDController::getP, turningPIDController::setP);
    builder.addDoubleProperty("Turning kI", turningPIDController::getI, turningPIDController::setI);
    builder.addDoubleProperty("Turning kD", turningPIDController::getD, turningPIDController::setD);
    builder.addDoubleProperty(
        "Turning kFF", turningPIDController::getFF, turningPIDController::setFF);
    builder.addDoubleProperty("Driving Vel (m/s)", drivingEncoder::getVelocity, null);
    builder.addDoubleProperty("Steering Pos (rad)", turningEncoder::getPosition, null);
    builder.addDoubleProperty("Desired Vel (m/s)", () -> desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty("Desired Steer (rad)", () -> desiredState.angle.getRadians(), null);
    builder.addBooleanProperty(
        "Turning encoder connected", turningAbsoluteEncoderChecker::encoderConnected, null);

  }

}
