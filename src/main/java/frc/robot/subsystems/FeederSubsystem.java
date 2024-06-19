package frc.robot.subsystems;

import static frc.robot.utils.SparkMaxUtils.check;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

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

    private RelativeEncoder feederEncoder;
    private SparkPIDController feederController;

    private boolean hasNoteFlag = false;
    private double feederPositionTarget = 0.0;

    private Integer isHoldingNoteCount = 0;

    public FeederSubsystem(){
        feederSpark = new CANSparkMax(Feeder.Feeder_CAN_Id, MotorType.kBrushless);
        beamBreakSensor = new DigitalInput(Feeder.INTIAL_BEAM_BREAK_PORT);
        //finalBeamBreakSensor = new DigitalInput(Feeder.FINAL_BEAM_BREAK_PORT);
        

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

        if(beamBreakSensor.get() && !hasNoteFlag){
            resetDistanceFromBeamBreak();
        }

        hasNoteFlag = beamBreakSensor.get();

        if (isHoldingNote() == false) {
            isHoldingNoteCount = 0;
        }

    }

    public void setFeederSpeed(double vel) {
        feederSpark.set(vel);
    }

    public boolean isHoldingNote(){
        isHoldingNoteCount++;
        return beamBreakSensor.get();
    }

    public boolean isHoldingNoteExtended(){
        return isHoldingNoteCount > 5;
    }

    public void resetDistanceFromBeamBreak(){
        feederEncoder.setPosition(0.0);
    }

    public double getDistanceFromBeamBreak(){ // Recorded in inches, not angle
        return feederEncoder.getPosition();
    }

    public boolean atFeederPosition(){
        return Math.abs(feederEncoder.getPosition() - feederPositionTarget) < Feeder.POSITION_TOLLERANCE;
    }

    public void setFeederPoseTarget(double position) {
        feederPositionTarget = position;
        feederController.setReference(position, ControlType.kPosition);
    }

    private boolean initSparks(){
        int errors = 0;
        errors += check(feederSpark.restoreFactoryDefaults());

        feederEncoder =  feederSpark.getEncoder();
        errors += check(feederEncoder.setPositionConversionFactor(Feeder.FEEDER_DISTANCE_PER_REV));

        feederController = feederSpark.getPIDController();
        feederController.setP(Feeder.kP);
        feederController.setI(Feeder.kI);
        feederController.setD(Feeder.kD);

        feederEncoder.setPosition(0.0);
        feederSpark.setSmartCurrentLimit(20);
        feederSpark.setIdleMode(IdleMode.kBrake);
        return errors == 0;
    }

    @Override
    public void initSendable(SendableBuilder builder){
        super.initSendable(builder);

        builder.addBooleanProperty("Is intial Beam Triggered", this::isHoldingNote, null);
        //builder.addBooleanProperty("Is final Beam Triggered", this::isFinalBeamTriggered, null);
 
        builder.addDoubleProperty("Feeder Current", feederSpark::getOutputCurrent, null);
        builder.addDoubleProperty("Feeder Pose", this::getDistanceFromBeamBreak, null);
        builder.addDoubleProperty("Target", () -> this.feederPositionTarget, null);
        builder.addBooleanProperty("At Position", this::atFeederPosition, null);
        builder.addDoubleProperty("Applied Output", feederSpark::getAppliedOutput, null);

        builder.addDoubleProperty("P Internal",() -> feederSpark.getPIDController().getP(), null);
        builder.addDoubleProperty("P External",() -> feederController.getP(), null);
        
    }

    
}
