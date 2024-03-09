package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightHelpers;

public class LimelightBlink extends Command{
    public LimelightBlink(){
        
    }
    
    @Override
    public void execute(){
        LimelightHelpers.setLEDMode_ForceBlink(Limelight._name);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
