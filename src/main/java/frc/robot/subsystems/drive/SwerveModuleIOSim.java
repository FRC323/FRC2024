package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private FlywheelSim driveSim;
  private SingleJointedArmSim turningSim;

  private PIDController turningPIDController;
  private PIDController drivingPIDController;
  private SimpleMotorFeedforward drivingFF;
  private double turningCommandVoltage;
  private double drivingCommandVoltage;
  private double direction = 1.0;

  private double _targetVel;
  private Rotation2d _targetAngle;
  private double _positionMeters = 0;

  public SwerveModuleIOSim(boolean isInverted) {
    this.direction = isInverted ? -1.0 : 1.0;
    driveSim =
        new FlywheelSim(
            DCMotor.getNEO(1),
            Constants.Swerve.Module.DRIVING_MOTOR_REDUCTION,
            Constants.Swerve.Module.WHEEL_MOI);
    turningSim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            Constants.Swerve.Module.STEERING_MOTOR_REDUCTION,
            0.001, // MOI - you could in theory get this from CAD?
            0.0, // Length in M
            Double.NEGATIVE_INFINITY, // min angle
            Double.POSITIVE_INFINITY, // max angle
            false, // We don't want to simulate gravity
            0.0 // Initial Angle
            );

    turningPIDController =
        new PIDController(
            Constants.Swerve.Module.TURNING_K_P,
            Constants.Swerve.Module.TURNING_K_I,
            Constants.Swerve.Module.TURNING_K_D);
    turningPIDController.enableContinuousInput(
        Constants.Swerve.Module.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS,
        Constants.Swerve.Module.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS);

    drivingPIDController =
        new PIDController(
            Constants.Swerve.Module.DRIVING_K_P,
            Constants.Swerve.Module.DRIVING_K_I,
            Constants.Swerve.Module.DRIVING_K_D);
    // TODO: I have no idea if setting KS to 1 is appropriate
    drivingFF = new SimpleMotorFeedforward(1.0, Constants.Swerve.Module.DRIVING_K_FF);
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    turningSim.setInput(turningCommandVoltage);
    driveSim.setInput(drivingCommandVoltage);
    turningSim.update(TimedRobot.kDefaultPeriod);
    driveSim.update(TimedRobot.kDefaultPeriod);

    inputs.turnOutputVolts = MathUtil.clamp(turningSim.getOutput(0), -12.0, 12.0);
    inputs.turnOutputCurrent = turningSim.getCurrentDrawAmps();
    inputs.turnEncoderPositionDeg +=
        Units.radiansToDegrees(turningSim.getVelocityRadPerSec()) * TimedRobot.kDefaultPeriod;
    inputs.turnAngleDeg = Units.radiansToDegrees(turningSim.getAngleRads());
    inputs.turnAngularVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(turningSim.getVelocityRadPerSec());

    inputs.driveOutputVolts = MathUtil.clamp(driveSim.getOutput(0), -12.0, 12.0);
    inputs.driveOutputCurrent = driveSim.getCurrentDrawAmps();
    inputs.drivePositionMeters +=
        driveSim.getAngularVelocityRadPerSec()
            * Constants.Swerve.Module.WHEEL_CIRCUMFERENCE_METERS
            * TimedRobot.kDefaultPeriod;
    inputs.driveVelocityMetersPerSecond =
        driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.Module.WHEEL_CIRCUMFERENCE_METERS;
  }

    @Override
  public void setCommandedOutputs(Rotation2d steeringAngle, double wheelVelocity) {
    turningCommandVoltage = turningPIDController.calculate(turningSim.getAngleRads(), steeringAngle.getRadians());
    drivingCommandVoltage =
        drivingPIDController.calculate(
            driveSim.getAngularVelocityRadPerSec()
                * Constants.Swerve.Module.WHEEL_CIRCUMFERENCE_METERS,
            wheelVelocity * direction) + drivingFF.calculate(wheelVelocity);
  _targetAngle = steeringAngle;
  _targetVel = wheelVelocity;
  }

  @Override
  public void resetDriveEncoder(double newValue) {
    _positionMeters = 0.0;
    driveSim.setState(newValue);
  }

  @Override
  public boolean initModule() {
    return true;
  }



  @Override
  public void periodic() {
    this._positionMeters +=
    driveSim.getAngularVelocityRadPerSec()
            * Constants.Swerve.Module.WHEEL_CIRCUMFERENCE_METERS
            * TimedRobot.kDefaultPeriod;
  }

  @Override
  public void burnFlashSparks() {
    // This method intentionally left blank
  }

  @Override
  public Rotation2d getModuleHeading() {
    return new Rotation2d(turningSim.getAngleRads());
  }

  @Override
  public double getVelocity() {
    return  driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.Module.WHEEL_CIRCUMFERENCE_METERS;
  }

  @Override
  public double getPosition() {
    return _positionMeters;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Driving kP", drivingPIDController::getP, drivingPIDController::setP);
    builder.addDoubleProperty("Driving kI", drivingPIDController::getI, drivingPIDController::setI);
    builder.addDoubleProperty("Driving kD", drivingPIDController::getD, drivingPIDController::setD);
    builder.addDoubleProperty(
            "Driving kFF", () -> 0, (val) -> {});
    builder.addDoubleProperty("Turning kP", turningPIDController::getP, turningPIDController::setP);
    builder.addDoubleProperty("Turning kI", turningPIDController::getI, turningPIDController::setI);
    builder.addDoubleProperty("Turning kD", turningPIDController::getD, turningPIDController::setD);
    builder.addDoubleProperty(
            "Driving kFF", () -> 0, null);
    builder.addDoubleProperty("Driving Vel (m/s)", () -> driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.Module.WHEEL_CIRCUMFERENCE_METERS, null);
    builder.addDoubleProperty("Steering Pos (rad)", turningSim::getAngleRads, null);
    builder.addDoubleProperty("Desired Vel (m/s)", () -> _targetVel, null);
    builder.addDoubleProperty("Desired Steer (rad)", () -> _targetAngle.getRadians(), null);
    builder.addBooleanProperty(
            "Turning encoder connected", () -> true, null);
  }
}
