package frc.robot.commands.SetCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMode;

public class SetVisionMode extends Command {
    private VisionSubsystem _visionSubsystem;
    private VisionMode _visionMode;

    public SetVisionMode(VisionSubsystem visionSubsystem, VisionMode mode) {
        _visionSubsystem = visionSubsystem;
        _visionMode = mode;
    }

    @Override
    public void execute() {
        _visionSubsystem.SetVisionMode(_visionMode);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
