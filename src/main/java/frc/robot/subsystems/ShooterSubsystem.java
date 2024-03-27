package frc.robot.subsystems;

import static frc.robot.utils.SparkMaxUtils.check;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Shooter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
  private SparkPIDController leftShooterController;
  private SparkPIDController rightShooterController;
  private RelativeEncoder leftShooterEncoder;
  private RelativeEncoder rightShooterEncoder;  

  private CANSparkMax leftShooterSpark;
  private CANSparkMax rightShooterSpark;

  private double targetShooterVelocity = 0.0;

  // TODO: Play with this number, aim is that it takes us ~1/4 to second spin up
  private SlewRateLimiter velocityRamp = new SlewRateLimiter(15000);
  private double shooterVelocity = 0;

  public ShooterSubsystem(){
    leftShooterSpark = new CANSparkMax(Shooter.Shooter_L_CAN_Id, MotorType.kBrushless);
    rightShooterSpark = new CANSparkMax(Shooter.Shooter_R_CAN_Id, MotorType.kBrushless);
    //Todo Add Right shooter spark

    leftShooterController = leftShooterSpark.getPIDController();
    rightShooterController = rightShooterSpark.getPIDController();

    leftShooterEncoder = leftShooterSpark.getEncoder ();
    rightShooterEncoder = rightShooterSpark.getEncoder();

    initSparks();

  }

  @Override
  public void periodic(){
    //    TODO: If ramping is causing issues, just set the references to targetVelocity
    if(targetShooterVelocity == 0.0){
      leftShooterController.setReference(0.0, com.revrobotics.CANSparkBase.ControlType.kVoltage);
      rightShooterController.setReference(0.0, com.revrobotics.CANSparkBase.ControlType.kVoltage);
    }else{
      leftShooterController.setReference(targetShooterVelocity, com.revrobotics.CANSparkBase.ControlType.kVelocity);
      rightShooterController.setReference(targetShooterVelocity, com.revrobotics.CANSparkBase.ControlType.kVelocity);
    }
  }

  public void setShooterSpeed(double vel) {
    targetShooterVelocity = vel;
  }


  public boolean atShootSpeed(){
    return 
      leftShooterEncoder.getVelocity() >= targetShooterVelocity * 0.95
      && rightShooterEncoder.getVelocity() >= targetShooterVelocity * 0.95;
  }

  private boolean initSparks() {
    int errors = 0;
    errors += check(leftShooterSpark.restoreFactoryDefaults());
    errors += check(rightShooterSpark.restoreFactoryDefaults());
     
    errors += check(rightShooterSpark.setSmartCurrentLimit(60));
    errors += check(leftShooterSpark.setSmartCurrentLimit(60));
    errors += check(rightShooterEncoder.setAverageDepth(2));
    errors += check(leftShooterEncoder.setAverageDepth(2));
    errors += check(rightShooterEncoder.setMeasurementPeriod(16));
    errors += check(leftShooterEncoder.setMeasurementPeriod(16));

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

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);

    builder.addDoubleProperty("ShooterVelocity L",leftShooterEncoder::getVelocity, null);
    builder.addDoubleProperty("ShooterVelocity R",rightShooterEncoder::getVelocity, null);
    builder.addDoubleProperty("Shooter Current L", leftShooterSpark::getOutputCurrent,null);
    builder.addDoubleProperty("Shooter Current R", rightShooterSpark::getOutputCurrent, null);

  }

public boolean isRunning() {
  return targetShooterVelocity != 0.0;
}

}
