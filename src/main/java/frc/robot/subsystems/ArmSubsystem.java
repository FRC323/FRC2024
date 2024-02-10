package frc.robot.subsystems;

import static frc.robot.utils.SparkMaxUtils.check;
import static frc.robot.utils.SparkMaxUtils.initWithRetry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.utils.NoRolloverEncoder;

public class ArmSubsystem extends SubsystemBase {
  // Control
  private ProfiledPIDController armController;
  private SimpleMotorFeedforward armFeedForward;

  private ProfiledPIDController shooterController;

  // Hardware
  private CANSparkMax leftSpark;
  private CANSparkMax rightSpark;
  private CANSparkMax shooterSpark;
  private CANSparkMax feederSpark;

  //    private AbsoluteEncoder armEncoder;
  private DutyCycleEncoder armAbsoluteEncoder;
  private double commandedVoltage = 0.0;

  //    private AbsoluteEncoderChecker encoderChecker;

  public ArmSubsystem() {
    leftSpark = new CANSparkMax(Constants.Arm.Arm_Actuation_L, MotorType.kBrushless);
    rightSpark = new CANSparkMax(Constants.Arm.Arm_Actuation_R, MotorType.kBrushless);
    shooterSpark = new CANSparkMax(Arm.Shooter.Shooter_CAN_Id, MotorType.kBrushless);
    feederSpark = new CANSparkMax(Arm.Shooter.Feeder_CAN_Id, MotorType.kBrushless);

    shooterController =
        new ProfiledPIDController(
            Arm.Shooter.kP, Arm.Shooter.kI, Arm.Shooter.kD, Arm.Shooter.CONSTRAINTS);

    armController = new ProfiledPIDController(Arm.kP, Arm.kI, Arm.kD, Arm.ARM_CONSTRAINTS);

    armFeedForward = new SimpleMotorFeedforward(0, Arm.kV, Arm.kA);
    armAbsoluteEncoder = new DutyCycleEncoder(Arm.ENCODER_PORT);
    initWithRetry(this::initSparks, 5);
  }

  public void storeArmOffset() {
    double offset = armAbsoluteEncoder.getAbsolutePosition();
    Preferences.setDouble(Arm.OFFSET_KEY, offset);
    armAbsoluteEncoder.setPositionOffset(offset);
  }

  public void setFeederSpeed(double vel) {
    feederSpark.set(vel);
  }

  public void setShooterSpeed(double vel) {
    shooterSpark.set(vel);
  }

  public double getArmAngleRads() {
    return armAbsoluteEncoder.get() * (-2 * Math.PI);
  }

  @Override
  public void periodic() {
    commandedVoltage = armController.calculate(getArmAngleRads())
    // TODO: If you re-enable this (and we should) it'll require a retune of the arm, punch it to
    // kp:0.1 and up slowly
    //            + armFeedForward.calculate(armController.getSetpoint().velocity)
    //            + (Arm.kG * Math.cos(getArmAngleRads()))
    ;
    leftSpark.setVoltage(commandedVoltage);
  }

  public void setTargetRads(double rads) {
    this.armController.setGoal(MathUtil.clamp(rads, Arm.SOFT_LIMIT_MIN, Arm.SOFT_LIMIT_MAX));
  }

  public boolean armIsAtTarget() {
    return armController.atGoal();
  }

  public void burnFlash() {
    Timer.delay(0.005);
    leftSpark.burnFlash();
    Timer.delay(0.005);
    rightSpark.burnFlash();
  }

  private boolean initSparks() {
    int errors = 0;
    // TODO: Config all
    errors += check(leftSpark.restoreFactoryDefaults());
    errors += check(rightSpark.restoreFactoryDefaults());
    errors += check(shooterSpark.restoreFactoryDefaults());
    errors += check(feederSpark.restoreFactoryDefaults());
    errors += check(rightSpark.follow(leftSpark, true));
    errors += check(leftSpark.setSmartCurrentLimit(Arm.CURRENT_LIMIT));
    return errors == 0;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("kP: ", armController::getP, armController::setP);
    builder.addDoubleProperty("kI: ", armController::getI, armController::setI);
    builder.addDoubleProperty("kD: ", armController::getD, armController::setD);

    //    builder.addDoubleProperty("Velocity (m/s)", armAbsoluteEncoder::getV, null);
    //    builder.addDoubleProperty(
    //        "Desired Velocity (m/s)",
    //        () -> {
    //          return armController.getGoal().velocity;
    //        },
    //        null);
    builder.addDoubleProperty("Position (rads)", this::getArmAngleRads, null);
    builder.addDoubleProperty("Absolute Position", armAbsoluteEncoder::getAbsolutePosition, null);
    builder.addDoubleProperty(
        "Desired Position (rads)",
        () -> {
          return armController.getGoal().position;
        },
        null);
    builder.addDoubleProperty("Arm Commanded Voltage", () -> commandedVoltage, null);
    builder.addDoubleProperty("Arm Current L", leftSpark::getOutputCurrent, null);
    builder.addDoubleProperty("Arm Current R", rightSpark::getOutputCurrent, null);
    builder.addBooleanProperty("Arm Encoder Plugged In", armAbsoluteEncoder::isConnected, null);
  }
}
