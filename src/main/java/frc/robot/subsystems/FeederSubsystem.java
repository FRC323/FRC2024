package frc.robot.subsystems;

import static frc.robot.utils.SparkMaxUtils.check;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Feeder;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightHelpers;


public class FeederSubsystem extends SubsystemBase{  
    private CANSparkMax feederSpark;
    private DigitalInput beamBreakSensor;
    //private DigitalInput finalBeamBreakSensor;

    private DutyCycleEncoder feederEncoder;
    private ProfiledPIDController feederController;

    public FeederSubsystem(){
        feederSpark = new CANSparkMax(Feeder.Feeder_CAN_Id, MotorType.kBrushless);
        beamBreakSensor = new DigitalInput(Feeder.INTIAL_BEAM_BREAK_PORT);
        //finalBeamBreakSensor = new DigitalInput(Feeder.FINAL_BEAM_BREAK_PORT);
        feederEncoder = new DutyCycleEncoder(Feeder.ENCODER_PORT);
        feederEncoder.setDistancePerRotation(Feeder.FEEDER_DISTANCE_PER_REV);
        feederEncoder.reset();

        feederController = new ProfiledPIDController(Feeder.kP, Feeder.kI, Feeder.kD, Feeder.FEEDER_CONSTRAINTS);
        feederController.setTolerance(0.01);

        initSparks();
    }

    @Override
    public void periodic(){
        if(isHoldingNote()){
            // PhotonPoseEstimatorSubsystem.backPhotonCamera.setLED(VisionLEDMode.kBlink);
            LimelightHelpers.setLEDMode_ForceBlink(Limelight._name);
        }else{
            LimelightHelpers.setLEDMode_ForceOff(Limelight._name);
        }
    }

    public void setFeederSpeed(double vel) {
        feederSpark.set(vel);
    }

    public boolean isHoldingNote(){
        return beamBreakSensor.get();
    }

    public void resetDistanceFromBeamBreak(){
        feederEncoder.reset();
    }

    public double getDistanceFromBeamBreak(){ // Recorded in inches, not angle
        return feederEncoder.getDistance();
    }

    public void adjustNote(){
        this.feederController.setGoal(getDistanceFromBeamBreak());
    }

    private boolean initSparks(){
        int errors = 0;
        errors += check(feederSpark.restoreFactoryDefaults());
        feederSpark.setIdleMode(IdleMode.kBrake);
        return errors == 0;
    }

    @Override
    public void initSendable(SendableBuilder builder){
        super.initSendable(builder);

        builder.addBooleanProperty("Is intial Beam Triggered", this::isHoldingNote, null);
        //builder.addBooleanProperty("Is final Beam Triggered", this::isFinalBeamTriggered, null);
 
        builder.addDoubleProperty("Feeder Current", feederSpark::getOutputCurrent, null);
    }
}
