package frc.robot.subsystems;

import static frc.robot.utils.SparkMaxUtils.check;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.utils.AbsoluteEncoderChecker;

public class ArmSubsystem extends SubsystemBase{
    //Control 
    private ProfiledPIDController armController;
    private SimpleMotorFeedforward feedforward;

    private ProfiledPIDController shooterController;

    //Hardware
    private CANSparkMax leftSpark;
    private CANSparkMax rightSpark;
    private CANSparkMax shooterSpark;
    private CANSparkMax feederSpark;

    private AbsoluteEncoder armEncoder;

    private AbsoluteEncoderChecker encoderChecker;

    public ArmSubsystem(){
        leftSpark = new CANSparkMax(Constants.Arm.Arm_Actuation_L, MotorType.kBrushless);       
        rightSpark = new CANSparkMax(Constants.Arm.Arm_Actuation_R, MotorType.kBrushless);

        shooterController = new ProfiledPIDController(
            Arm.Shooter.kP,Arm.Shooter.kI,Arm.Shooter.kD,Arm.Shooter.CONSTRAINTS
        );

        armController = new ProfiledPIDController(
            Arm.kP,Arm.kI,Arm.kD,Arm.ARM_CONSTRAINTS);

        feedforward = new SimpleMotorFeedforward(0, 0.0);

        armEncoder = leftSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        encoderChecker = new AbsoluteEncoderChecker();
        

        initSparks();
    }

    @Override
    public void periodic(){
        leftSpark.set(
            armController.calculate(armEncoder.getPosition())
            + feedforward.calculate(armController.getSetpoint().velocity)
            + (Arm.kG * Math.cos(armEncoder.getPosition()))
        );
        
        encoderChecker.addReading(armEncoder.getPosition());
    
        // shooterSpark.set(

        // )
        
    }

    public void setTargetAngle(double angle){
        this.armController.setGoal(angle);
    }

    public double getCurrentAngle(){
        return this.armEncoder.getPosition();
    }

    public void burnFlash(){
        Timer.delay(0.005);
        leftSpark.burnFlash();
        Timer.delay(0.005);
        rightSpark.burnFlash();     
    }

    private void initSparks(){
        int errors = 0;
        //TODO: Config all
        
        errors += check(
            leftSpark.restoreFactoryDefaults()
        );
        
        errors += check(rightSpark.follow(leftSpark,true));
        
        errors += check(leftSpark.setSmartCurrentLimit(Arm.CURRENT_LIMIT));


        
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("kP: ",armController::getP, armController::setP);
        builder.addDoubleProperty("kI: ",armController::getI, armController::setI);            
        builder.addDoubleProperty("kD: ",armController::getD, armController::setD);
    
        builder.addDoubleProperty("Velocity (m/s)", armEncoder::getVelocity,null);
        builder.addDoubleProperty("Desired Velocity (m/s)", () -> {return armController.getGoal().velocity;}, null);
        builder.addDoubleProperty("Position (rads)",armEncoder::getPosition,null);
        builder.addDoubleProperty("Desired Position (rads)", () -> {return armController.getGoal().position;}, null);
    }


}
