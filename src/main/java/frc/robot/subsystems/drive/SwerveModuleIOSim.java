package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private FlywheelSim driveSim;
  private SingleJointedArmSim turningSim;

  private PIDController turningController;
  private PIDController driveController;
  private double turningCommandVoltage;
  private double drivignCommandVoltage;
  private double direction = 1.0;

  public SwerveModuleIOSim(boolean isInverted) {
    this.direction = isInverted?-1.0:1.0;
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

    turningController =
        new PIDController(
            Constants.Swerve.Module.TURNING_K_P,
            Constants.Swerve.Module.TURNING_K_I,
            Constants.Swerve.Module.TURNING_K_D,
            Constants.Swerve.Module.TURNING_K_FF);
    turningController.enableContinuousInput(
        Constants.Swerve.Module.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS,
        Constants.Swerve.Module.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS);
    driveController =
        new PIDController(
            Constants.Swerve.Module.DRIVING_K_P,
            Constants.Swerve.Module.DRIVING_K_I,
            Constants.Swerve.Module.DRIVING_K_D,
            Constants.Swerve.Module.DRIVING_K_FF);
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    turningSim.setInput(turningCommandVoltage);
    driveSim.setInput(drivignCommandVoltage);
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
  public void setCommandedOutputs(double steeringAngle, double wheelVelocity) {
    turningCommandVoltage = turningController.calculate(turningSim.getAngleRads(), steeringAngle);
    drivignCommandVoltage =
        driveController.calculate(
            driveSim.getAngularVelocityRadPerSec()
                * Constants.Swerve.Module.WHEEL_CIRCUMFERENCE_METERS,
            wheelVelocity * direction);
  }

  @Override
  public void resetSteeringEncoder(double newValue) {
    turningSim.setState(newValue, turningSim.getVelocityRadPerSec());
  }

  @Override
  public void resetDriveEncoder(double newValue) {
    driveSim.setState(newValue);
  }

  @Override
  public boolean initModule() {
    return true;
  }



  @Override
  public void periodic() {}

  @Override
  public void burnFlashSparks() {
    // This method intentionally left blank
  }
}
