package frc.robot.subsystems;

import static frc.robot.utils.SparkMaxUtils.check;
import static frc.robot.utils.SparkMaxUtils.initWithRetry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Feeder;
import frc.robot.Constants.Shooter;
import frc.robot.utils.NoRolloverEncoder;

public class ArmSubsystem extends SubsystemBase {
  // Control
  private ProfiledPIDController armController;
  private ArmFeedforward armFeedForward;

  // Hardware
  private CANSparkMax leftSpark;
  private CANSparkMax rightSpark;


  //    private AbsoluteEncoder armEncoder;
  private DutyCycleEncoder armAbsoluteEncoder;
  private double commandedVoltage = 0.0;
  private boolean voltageOveride = false;
  //    private AbsoluteEncoderChecker encoderChecker;

  public ArmSubsystem() {
    leftSpark = new CANSparkMax(Constants.Arm.Arm_Actuation_L, MotorType.kBrushless);
    rightSpark = new CANSparkMax(Constants.Arm.Arm_Actuation_R, MotorType.kBrushless);
    armController = new ProfiledPIDController(Arm.kP, Arm.kI, Arm.kD, Arm.ARM_CONSTRAINTS);
    armController.setTolerance(0.01);

    armFeedForward = new ArmFeedforward(0, Arm.kG, Arm.kV);
    armAbsoluteEncoder = new DutyCycleEncoder(Arm.ENCODER_PORT);
    armAbsoluteEncoder.setPositionOffset(Preferences.getDouble(Arm.OFFSET_KEY, 0.0));


    initWithRetry(this::initSparks, 5);
    armController.setGoal(getArmAngleRads());
  }

  public void storeArmOffset() {
    armAbsoluteEncoder.reset();
    Preferences.setDouble(Arm.OFFSET_KEY, armAbsoluteEncoder.getPositionOffset());
  }

  

  public double getArmAngleRads() {
    return armAbsoluteEncoder.get() * (2 * Math.PI);
  }

  @Override
  public void periodic() {
    if(!voltageOveride){
      commandedVoltage = armController.calculate(getArmAngleRads())
                + armFeedForward.calculate(getArmAngleRads(), armController.getSetpoint().velocity);
    }
    leftSpark.setVoltage(commandedVoltage); 
  }

  public void setTargetRads(double rads) {
    this.armController.setGoal(MathUtil.clamp(rads, Arm.SOFT_LIMIT_MIN, Arm.SOFT_LIMIT_MAX));
    voltageOveride = false;
  }

  public void setArmPower(double power){
    voltageOveride = true;
    leftSpark.set(power);
  }

  public double getArmTarget(){
    return armController.getGoal().position;
  }
  
  public boolean armIsAtTarget() {
    var target = armController.getGoal();
    return Math.abs(target.position - getArmAngleRads()) < Arm.AT_TARGET_TOLLERANCE;
    // return armController.atGoal();
  }

  public boolean armTargetValidSpeakerTarget(){
    var goal = armController.getGoal().position;
    return goal != Arm.ARM_AMP_POSE
    && goal != Arm.ARM_HANDOFF_POSE
    && goal != Arm.ARM_DOWN_POSE
    && goal != Arm.ARM_INTAKE_UNFOLDING_POSE
    && goal != Arm.ARM_HUMAN_PLAYER_POSE
    && goal != Arm.ARM_CLIMB_POSE;
  } 

  public void burnFlash() {
    Timer.delay(0.005);
    leftSpark.burnFlash();
    Timer.delay(0.005);
    rightSpark.burnFlash();
  }

  private boolean initSparks() {
    int errors = 0;
    errors += check(leftSpark.restoreFactoryDefaults());
    errors += check(rightSpark.restoreFactoryDefaults());
   errors += check(rightSpark.follow(leftSpark, true));
    leftSpark.setInverted(true); //TODO: Add to constant
    // errors += check(rightShooterSpark.follow(leftShooterSpark,false));
    errors += check(leftSpark.setSmartCurrentLimit(Arm.CURRENT_LIMIT));
    errors += check(leftSpark.setIdleMode(IdleMode.kBrake));
    errors += check(rightSpark.setIdleMode(IdleMode.kBrake));

    return errors == 0;

  }

  public void enableCoast(){
    leftSpark.setIdleMode(IdleMode.kCoast);
    rightSpark.setIdleMode(IdleMode.kCoast);
  }

  public void enableBrake(){
    leftSpark.setIdleMode(IdleMode.kBrake);
    rightSpark.setIdleMode(IdleMode.kBrake);
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
