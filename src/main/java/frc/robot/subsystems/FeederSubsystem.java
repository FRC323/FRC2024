package frc.robot.subsystems;

import static frc.robot.utils.SparkMaxUtils.check;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Feeder;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightHelpers;;


public class FeederSubsystem extends SubsystemBase{  
    private CANSparkMax feederSpark;
    private DigitalInput beamBreakSensor;

    public FeederSubsystem(){
        feederSpark = new CANSparkMax(Feeder.Feeder_CAN_Id, MotorType.kBrushless);
        beamBreakSensor = new DigitalInput(Feeder.BEAM_BREAK_PORT);
    }

    @Override
    public void periodic(){
        if(!isHoldingNote()){
        LimelightHelpers.setLEDMode_ForceOff(Limelight._name);
        }
    }

    public void setFeederSpeed(double vel) {
        feederSpark.set(vel);
    }

    public boolean isHoldingNote(){
        return beamBreakSensor.get();
    }

    private boolean initSparks(){
        int errors = 0;
        errors += check(feederSpark.restoreFactoryDefaults());
        return errors == 0;
    }

    @Override
    public void initSendable(SendableBuilder builder){
        super.initSendable(builder);

        builder.addBooleanProperty("Beam Blocked", ()-> beamBreakSensor.get(), null);
        builder.addBooleanProperty("Is Holding Note", this::isHoldingNote, null);
 
        builder.addDoubleProperty("Feeder Current", feederSpark::getOutputCurrent, null);
    }
}