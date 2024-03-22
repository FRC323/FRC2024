package frc.robot.commands.SetCommands;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;

public class SetLimelightBlink extends Command{
    
    @Override
    public void execute(){
        PoseEstimatorSubsystem.backPhotonCamera.setLED(VisionLEDMode.kBlink);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
