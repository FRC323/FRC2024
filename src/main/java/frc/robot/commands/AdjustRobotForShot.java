package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.LinearInterpolator;


public class AdjustRobotForShot extends Command{
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private VisionSubsystem visionSubsystem;

    private final LinearInterpolator ArmAngleInterpolation = new LinearInterpolator(new ArrayList<>(){
        
    });

    private final LinearInterpolator NoteFeedforward = new LinearInterpolator(new ArrayList<>(){
        
    });
   
    public AdjustRobotForShot(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void execute(){
        var optionalCapture = visionSubsystem.getLimelightCapture();
        if(!optionalCapture.isPresent()) return;
        var _limelightcapture = optionalCapture.get();
        if(_limelightcapture.hasTarget()){
            armSubsystem.setTargetRads(getArmAngle(_limelightcapture.robotpose_targetspace()[5]));
        }
    }

    public double getArmAngle(double distanceToGoal){
        return ArmAngleInterpolation.get(distanceToGoal)
            + NoteFeedforward.get(distanceToGoal);

    }

    public double getRobotOffsetAngle(){
        return 0.0;
    }
}
