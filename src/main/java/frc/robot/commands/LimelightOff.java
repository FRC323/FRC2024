package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightHelpers;

public class LimelightOff extends Command{
    @Override
    public void execute() {
        LimelightHelpers.setLEDMode_ForceOff(Limelight._name);
    }
}
