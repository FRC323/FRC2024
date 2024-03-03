package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.LinearInterpolator;


public class AdjustArmForShot extends Command{
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private VisionSubsystem visionSubsystem;

    private static final InterpolatingDoubleTreeMap armAngleInterpolation = new InterpolatingDoubleTreeMap();

    public AdjustArmForShot(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
        this.visionSubsystem = visionSubsystem;
    
        initializeInterpolator();
    }

    @Override
    public void execute(){
        var optionalCapture = visionSubsystem.getLimelightCapture();
        if(!optionalCapture.isPresent()) return;
        var _limelightcapture = optionalCapture.get();
        if(_limelightcapture.hasTarget()){
            var targetDistanceOptinal = visionSubsystem.getTargetDistance();
            if(!targetDistanceOptinal.isPresent()) return;
            var targetDisance = targetDistanceOptinal.getAsDouble();
            armSubsystem.setTargetRads(
                getArmAngle(
                    targetDisance
                ));
        }
    }

    public double getArmAngle(double distanceToGoal){
        return armAngleInterpolation.get(distanceToGoal);
    }

    public double getRobotOffsetAngle(){
        return 0.0;
    }

    private void initializeInterpolator(){
        // armAngleInterpolation.put(0.0,0.0);
        // armAngleInterpolation.put(1.47,-0.232);
        // armAngleInterpolation.put( , );
    }   
}
