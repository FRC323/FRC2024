package frc.robot.subsystems.drive;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import com.revrobotics.CANSparkLowLevel.MotorType;
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
  private final double moduleOffset;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d(0));

  public SwerveModule(int drivingCanId, int turningCanId, double moduleOffset) {
    drivingSpark = new CANSparkMax(drivingCanId, MotorType.kBrushless);
    turningSpark = new CANSparkMax(turningCanId, MotorType.kBrushless);
    this.moduleOffset = moduleOffset;

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
    SparkPIDController drivingPidTmp = turningSpark.getPIDController();
    errors += check(drivingPidTmp.setFeedbackDevice(drivingEncoderTmp));
    errors +=
        check(drivingEncoderTmp.setInverted(Constants.Swerve.Module.DRIVING_ENCODER_INVERTED));
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

    errors += check(turningSpark.setIdleMode(Constants.Swerve.Module.DRIVING_MOTOR_IDLE_MODE));
    errors +=
        check(
            turningSpark.setSmartCurrentLimit(
                Constants.Swerve.Module.DRIVING_MOTOR_CURRENT_LIMIT_AMPS));
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
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(moduleOffset));
    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(
            correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));
    this.desiredState = optimizedState;
    drivingPIDController.setReference(
        optimizedState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
    turningPIDController.setReference(
        optimizedState.angle.getRadians(), CANSparkBase.ControlType.kPosition);
  }

  public double getEncoderAbsPositionRad() {
    return turningEncoder.getPosition();
  }

  public void periodic() {
    turningAbsoluteEncoderChecker.addReading(turningEncoder.getPosition());
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
