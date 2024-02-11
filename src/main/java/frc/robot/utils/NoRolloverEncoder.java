package frc.robot.utils;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class NoRolloverEncoder{
    private DutyCycleEncoder encoder;
    private double offset;
    
    public NoRolloverEncoder(int encoderPort,double offset){
        encoder = new DutyCycleEncoder(encoderPort);
        this.offset = offset;
    }

    public double get(){
        return adjustAngleForRollover(encoder.getAbsolutePosition()) - adjustAngleForRollover(offset); 
    }

    public double getAbsolutePosition(){
        return adjustAngleForRollover(encoder.getAbsolutePosition());
    }

    public void reset(){
        this.offset = encoder.getAbsolutePosition();
    }

    public void setOffset(double offset){
        this.offset = adjustAngleForRollover(offset);
    }

    public double getOffset(){
        return adjustAngleForRollover(this.offset);
    }
    
    public boolean isConnected(){
        return encoder.isConnected();
    }

    private double adjustAngleForRollover(double angle){
            double reportedPosition = angle;
            return reportedPosition > 0.5 ? 
                reportedPosition - 0.5
                : reportedPosition + 0.5;
    }
}
