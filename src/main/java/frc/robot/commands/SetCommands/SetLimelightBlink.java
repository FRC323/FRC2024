package frc.robot.commands.SetCommands;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightHelpers;

public class SetLimelightBlink extends Command{
    
    @Override
    public void execute(){
        // PhotonPoseEstimatorSubsystem.backPhotonCamera.setLED(VisionLEDMode.kBlink);
        LimelightHelpers.setLEDMode_ForceBlink(Limelight._name);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
