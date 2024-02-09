package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.utils.NoRolloverEncoder;

import static frc.robot.utils.SparkMaxUtils.check;
import static frc.robot.utils.SparkMaxUtils.initWithRetry;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax feederRoller;
    private CANSparkMax wristSpark;

    private NoRolloverEncoder wristAbsoluteEncoder;
    private double commandedVoltage = 0;
    private double armOffset;

    private ProfiledPIDController wristController;
    

    public IntakeSubsystem(){
        feederRoller = new CANSparkMax(Constants.Intake.ROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
        wristSpark = new CANSparkMax(Constants.Intake.ACTUATION_ID, CANSparkLowLevel.MotorType.kBrushless);
        wristAbsoluteEncoder = new NoRolloverEncoder(Intake.ENCODER_PORT,Preferences.getDouble(Intake.OFFSET_KEY, 0.0));
        wristController = new ProfiledPIDController(Intake.kP, Intake.kI, Intake.kD,Intake.WRIST_CONSTRAINTS);

        initWithRetry(this::initSparks, 5);
    }

    public void storeWristOffset() {
        wristAbsoluteEncoder.reset();
        Preferences.setDouble(Intake.OFFSET_KEY, wristAbsoluteEncoder.getOffset());
    }

    public double getWristAngleRads() {
        return wristAbsoluteEncoder.get() * (-2 * Math.PI) * Intake.ENCODER_GEAR_RATION;
    }

    @Override
    public void periodic(){
        
        commandedVoltage = wristController.calculate(getWristAngleRads());
        //todo: Implement Feedforward at the same time as the arm
        this.wristSpark.set(commandedVoltage);
    }

    public void setIntakeSpeed(double vel) {
        this.feederRoller.set(vel); 
    }

    public void setTargetRads(double rads) {
        this.wristController.setGoal(MathUtil.clamp(rads, Intake.SOFT_LIMIT_MIN, Intake.SOFT_LIMIT_MAX));
    }

    public boolean wristIsAtTarget() {
        return wristController.atGoal();
    }
    public void burnFlash() {
        Timer.delay(0.005);
        wristSpark.burnFlash();
        Timer.delay(0.005);
        feederRoller.burnFlash();
    }

    public boolean initSparks() {
        int errors = 0;

        errors += check(feederRoller.restoreFactoryDefaults());
        errors += check(wristSpark.restoreFactoryDefaults());
        errors += check(wristSpark.setSmartCurrentLimit(Intake.CURRENT_LIMIT));
        // wristAbsoluteEncoder.setDutyCycleRange(errors, errors);
        // wristAbsoluteEncoder.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);

        return errors == 0;
    }
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("kP: ", wristController::getP, wristController::setP);
    builder.addDoubleProperty("kI: ", wristController::getI, wristController::setI);
    builder.addDoubleProperty("kD: ", wristController::getD, wristController::setD);

    builder.addDoubleProperty("Position (rads)", this::getWristAngleRads, null);
    builder.addBooleanProperty("AtGoal", wristController::atGoal, null);
    builder.addDoubleProperty("AbsolutEncoder", () -> wristAbsoluteEncoder.getAbsolutePosition(), null);
    builder.addDoubleProperty(
        "Desired Position (rads)",
        () -> {
          return wristController.getGoal().position;
        },
        null);
    builder.addDoubleProperty("Arm Commanded Voltage", () -> commandedVoltage, null);
    builder.addDoubleProperty("Arm Current L", wristSpark::getOutputCurrent, null);
    // builder.addBooleanProperty("Arm Encoder Plugged In", wristAbsoluteEncoder::isConnected, null);
  }


}