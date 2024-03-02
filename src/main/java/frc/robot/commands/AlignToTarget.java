package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.Functions;

public class AlignToTarget extends Command {
    private VisionSubsystem _visionSubsystem;
    private DriveSubsystem _driveSubsystem;
    private CommandJoystick driveStick;
    
    private final double ROTATION_kP = 0.05;
    private final double ROTATION_kI = 0.0;
    private final double ROTATION_kD = 0.0;
    private PIDController rotController = new PIDController(ROTATION_kP, ROTATION_kI, ROTATION_kD);
    private int[] _requestedAprilTagIds;

    public AlignToTarget(VisionSubsystem visionSubsystem, 
        DriveSubsystem driveSubsystem,
        double targetHeight,
        int[] requestedAprilTagIds) {

        addRequirements(driveSubsystem, visionSubsystem);
        this._visionSubsystem = visionSubsystem;
        this._driveSubsystem = driveSubsystem;
        this._requestedAprilTagIds = requestedAprilTagIds;
        this.driveStick = driveStick;

        rotController.setTolerance(Units.degreesToRadians(5.0));
    }

    @Override
    public void execute() {
        var optionalCapture = _visionSubsystem.getLimelightCapture();
        if(!optionalCapture.isPresent()) return;
        var limelightCapture = optionalCapture.get();


        if (limelightCapture.hasTarget() && Functions.contains(this._requestedAprilTagIds, (int)limelightCapture.aprilTagId()))
        {   
            rotController.setSetpoint(limelightCapture.robotpose_targetspace()[5]);

            _driveSubsystem.drive(
                -driveStick.getY(), 
                -driveStick.getX(), 
                MathUtil.clamp(rotController.calculate(_driveSubsystem.getRobotPose2d().getRotation().getRadians()),-0.03,0.03),
                true
                );
        }else{
            _driveSubsystem.drive(-driveStick.getY(), -driveStick.getX(), 0.0, true);
        }
    }

}
