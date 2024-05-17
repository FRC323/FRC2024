package frc.robot.subsystems;

import static frc.robot.utils.SparkMaxUtils.check;

import org.photonvision.common.hardware.VisionLEDMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Feeder;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightHelpers;


public class FeederSubsystem extends SubsystemBase{  
    private CANSparkMax feederSpark;
    private DigitalInput intialBeamBreakSensor;
    private DigitalInput finalBeamBreakSensor;

    public FeederSubsystem(){
        feederSpark = new CANSparkMax(Feeder.Feeder_CAN_Id, MotorType.kBrushless);
        intialBeamBreakSensor = new DigitalInput(Feeder.INTIAL_BEAM_BREAK_PORT);
        finalBeamBreakSensor = new DigitalInput(Feeder.FINAL_BEAM_BREAK_PORT);

        initSparks();
    }

    @Override
    public void periodic(){
        if(isIntialBeamTriggered()){
            // PhotonPoseEstimatorSubsystem.backPhotonCamera.setLED(VisionLEDMode.kBlink);
            LimelightHelpers.setLEDMode_ForceBlink(Limelight._name);
        }else{
            LimelightHelpers.setLEDMode_ForceOff(Limelight._name);
        }
    }

    public void setFeederSpeed(double vel) {
        feederSpark.set(vel);
    }

    public boolean isIntialBeamTriggered(){
        return intialBeamBreakSensor.get();
    }

    public boolean isFinalBeamTriggered(){
        return finalBeamBreakSensor.get();
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

        builder.addBooleanProperty("Intial Beam Blocked", ()-> intialBeamBreakSensor.get(), null);
        builder.addBooleanProperty("Final beam Blocked", ()-> finalBeamBreakSensor.get(), null);
        builder.addBooleanProperty("Is intial Beam Triggered", this::isIntialBeamTriggered, null);
        builder.addBooleanProperty("Is final Beam Triggered", this::isFinalBeamTriggered, null);
 
        builder.addDoubleProperty("Feeder Current", feederSpark::getOutputCurrent, null);
    }
}
