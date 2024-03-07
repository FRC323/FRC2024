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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Arm.Shooter;
import frc.robot.utils.NoRolloverEncoder;

public class ArmSubsystem extends SubsystemBase {
  // Control
  private ProfiledPIDController armController;
  private ArmFeedforward armFeedForward;

  private SparkPIDController leftShooterController;
  private SparkPIDController rightShooterController;
  private RelativeEncoder leftShooterEncoder;
  private RelativeEncoder rightShooterEncoder;

  // Hardware
  private CANSparkMax leftSpark;
  private CANSparkMax rightSpark;
  private CANSparkMax leftShooterSpark;
  private CANSparkMax rightShooterSpark;
  private CANSparkMax feederSpark;

  //    private AbsoluteEncoder armEncoder;
  private DutyCycleEncoder armAbsoluteEncoder;
  private DigitalInput beamBreakSensor;
  private double commandedVoltage = 0.0;
  private boolean voltageOveride = false;
  private double targetShooterVelocity = 0.0;

  //    private AbsoluteEncoderChecker encoderChecker;

  public ArmSubsystem() {
    leftSpark = new CANSparkMax(Constants.Arm.Arm_Actuation_L, MotorType.kBrushless);
    rightSpark = new CANSparkMax(Constants.Arm.Arm_Actuation_R, MotorType.kBrushless);
    leftShooterSpark = new CANSparkMax(Arm.Shooter.Shooter_L_CAN_Id, MotorType.kBrushless);
    rightShooterSpark = new CANSparkMax(Arm.Shooter.Shooter_R_CAN_Id, MotorType.kBrushless);
    //Todo Add Right shooter spark
    feederSpark = new CANSparkMax(Arm.Shooter.Feeder_CAN_Id, MotorType.kBrushless);

    leftShooterController = leftShooterSpark.getPIDController();
    rightShooterController = rightShooterSpark.getPIDController();

    leftShooterEncoder = leftShooterSpark.getEncoder ();
    rightShooterEncoder = rightShooterSpark.getEncoder();

    armController = new ProfiledPIDController(Arm.kP, Arm.kI, Arm.kD, Arm.ARM_CONSTRAINTS);
    armController.setTolerance(0.20);

    armFeedForward = new ArmFeedforward(0, Arm.kG, Arm.kV);
    armAbsoluteEncoder = new DutyCycleEncoder(Arm.ENCODER_PORT);
    armAbsoluteEncoder.setPositionOffset(Preferences.getDouble(Arm.OFFSET_KEY, 0.0));

    beamBreakSensor = new DigitalInput(Arm.BEAM_BREAK_PORT);

    initWithRetry(this::initSparks, 5);
    armController.setGoal(getArmAngleRads());
  }

  public void storeArmOffset() {
    armAbsoluteEncoder.reset();
    Preferences.setDouble(Arm.OFFSET_KEY, armAbsoluteEncoder.getPositionOffset());
  }

  public void setFeederSpeed(double vel) {
    feederSpark.set(vel);
  }

  public void setShooterSpeed(double vel) {
    leftShooterController.setReference(vel,ControlType.kVelocity);
    rightShooterController.setReference(vel,ControlType.kVelocity);
    targetShooterVelocity = vel;
  }

  public double getArmAngleRads() {
    return armAbsoluteEncoder.get() * (2 * Math.PI);
  }

  @Override
  public void periodic() {
    if(!voltageOveride){
      commandedVoltage = armController.calculate(getArmAngleRads())
      // TODO: If you re-enable this (and we should) it'll require a retune of the arm, punch it to
      // kp:0.1 and up slowly
                + armFeedForward.calculate(getArmAngleRads(), armController.getSetpoint().velocity)
      //            + (Arm.kG * Math.cos(getArmAngleRads())
      ;
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
    return armController.atGoal();
  }

  public boolean atShootSpeed(){
    return atShootSpeed(targetShooterVelocity);
  }

  public boolean atShootSpeed(double shooterRPM){
    return 
      leftShooterEncoder.getVelocity() >= shooterRPM
      && rightShooterEncoder.getVelocity() >= shooterRPM; 
  }

  public boolean isHoldingNote(){
    return beamBreakSensor.get();
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
    errors += check(leftShooterSpark.restoreFactoryDefaults());
    errors += check(rightShooterSpark.restoreFactoryDefaults());
    errors += check(feederSpark.restoreFactoryDefaults());
    errors += check(rightSpark.follow(leftSpark, true));
    leftSpark.setInverted(true); //TODO: Add to constant
    // errors += check(rightShooterSpark.follow(leftShooterSpark,false));
    errors += check(leftSpark.setSmartCurrentLimit(Arm.CURRENT_LIMIT));
    // errors += check(leftSpark.setIdleMode(IdleMode.kBrake));
    // errors += check(rightSpark.setIdleMode(IdleMode.kBrake));

    errors += check(rightShooterSpark.setSmartCurrentLimit(60));
    errors += check(leftShooterSpark.setSmartCurrentLimit(60));

    errors += check(rightShooterController.setFF(Shooter.kF));
    errors += check(rightShooterController.setP(Shooter.kP));   
    errors += check(rightShooterController.setI(Shooter.kI));
    errors += check(rightShooterController.setD(Shooter.kD));

    errors += check(leftShooterController.setFF(Shooter.kF));
    errors += check(leftShooterController.setP(Shooter.kP));   
    errors += check(leftShooterController.setI(Shooter.kI));
    errors += check(leftShooterController.setD(Shooter.kD));

    leftShooterSpark.setInverted(true);
    rightShooterSpark.setInverted(true);

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

    builder.addBooleanProperty("Beam Blocked", ()-> beamBreakSensor.get(), null);
    builder.addBooleanProperty("Is Holding Note", this::isHoldingNote, null);
    builder.addDoubleProperty("ShooterVelocity L",leftShooterEncoder::getVelocity, null);
    builder.addDoubleProperty("ShooterVelocity R",rightShooterEncoder::getVelocity, null);
    builder.addDoubleProperty("Shooter Current L", leftShooterSpark::getOutputCurrent,null);
    builder.addDoubleProperty("Shooter Current R", rightShooterSpark::getOutputCurrent, null);

    builder.addDoubleProperty("Feeder Current", feederSpark::getOutputCurrent, null);
  }
}
