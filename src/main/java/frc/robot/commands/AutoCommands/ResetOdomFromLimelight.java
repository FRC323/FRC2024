package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;

public class ResetOdomFromLimelight extends Command{
    private PoseEstimatorSubsystem poseEstimatorSubsystem;
    private boolean finished = false;

    public ResetOdomFromLimelight(PoseEstimatorSubsystem poseEstimatorSubsystem){
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        addRequirements(poseEstimatorSubsystem);
    }

    @Override
    public void execute(){
        // if(poseEstimatorSubsystem.getEstimatedPosition().getRotation().getRadians() != 0.0){
            poseEstimatorSubsystem.updateOdometry();
            finished = true;
        // }
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}
