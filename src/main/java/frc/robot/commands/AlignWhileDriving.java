package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.ShotState;

public class AlignWhileDriving extends Command{
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private VisionSubsystem visionSubsystem;
    private CommandJoystick driveStick;

    private DoubleSupplier vx,vy;

    private ShotState shotState;
    
    public AlignWhileDriving(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier vx, DoubleSupplier vy){
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.driveStick = driveStick;

        this.vx = vx;
        this.vy = vy;
        
    }


    @Override
    public void execute(){

        updateShotState();

        if(shotState == null) return;

        //Todo: Follow Heading
        driveSubsystem.drive(
            vx.getAsDouble(),
            vy.getAsDouble(),
            0.0,
            true
        );


        armSubsystem.setTargetRads(
            shotState.get_armAngle().getRadians()
        );


        armSubsystem.setShooterSpeed(
            Constants.Arm.Shooter.SHOOTER_SPEED
        );

        


    }
    
    private void updateShotState(){
        var optionalCapture = visionSubsystem.getLimelightCapture();
        if(optionalCapture.isEmpty()) return;
        var limelightCapture = optionalCapture.get();
        var shotTarget = new Translation2d(
            limelightCapture.robotpose_targetspace()[0],
            limelightCapture.robotpose_targetspace()[1]
        );

        
        //Range To Target
        var optionalRange = VisionSubsystem.getTargetDistance();
        if(optionalRange.isEmpty()) return;
        var rangeToTarget = optionalRange.getAsDouble();

        //Robot Pose
        var robotPose = driveSubsystem.getRobotPose2d();

        //Robot Velocity
        var robotVelocity = driveSubsystem.getChassisSpeed();

        //dt (Todo: find actual dt)
        var dt = 1.0;

        this.shotState = ShotState.computedFromPose(
            shotTarget,
            rangeToTarget,
            robotPose,
            robotVelocity,
            dt
        );
    }

    @Override
    public void end(boolean interupted){
        armSubsystem.setShooterSpeed(0);
        armSubsystem.setFeederSpeed(0);
        armSubsystem.setTargetRads(armSubsystem.getArmAngleRads());
    }
}
