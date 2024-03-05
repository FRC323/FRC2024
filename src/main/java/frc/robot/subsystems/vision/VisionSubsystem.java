package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight.LimelightCaptureDetail;
import frc.robot.utils.ShotState;

public class VisionSubsystem extends SubsystemBase {
    private final Limelight _limelight = new Limelight();
    private static LimelightCaptureDetail _limelightCapture;

    private static Optional<ShotState> shotState = Optional.empty();

    private DriveSubsystem driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
    }

    public Optional<LimelightCaptureDetail> getLimelightCapture() {
        //TODO: Find a way to handle null limelight captures throughout the whol program
        if(_limelightCapture == null) return Optional.empty();
        return Optional.of(_limelightCapture);
    }
    
    public static OptionalDouble getTargetDistance(){
        if(_limelightCapture == null) return OptionalDouble.empty();
        double tagHeight = Constants.AprilTags.Speaker.HEIGHT;
        double lensHeight = Constants.Vision.LIMELIGHT_LENS_HEIGHT_INCHES;
        double goalAngleRads = Units.degreesToRadians(Constants.Vision.LIMELIGHT_MOUNT_ANGLE_DEGREES + _limelightCapture.yOffset());
        return OptionalDouble.of((tagHeight - lensHeight) / Math.tan(goalAngleRads));

    }

    public static Optional<ShotState> getShotState(){
        return shotState;
    }

    @Override
    public void periodic() {
        try {
            _limelightCapture = _limelight.capture();
        }catch(Exception e){    
            System.out.println("Limelight Exception: " + e.toString());
            // System.out.println("No alliance info available for botpose");
        }

        computeShotState(driveSubsystem);

        
    }

    public void computeShotState(DriveSubsystem driveSubsystem){
        //Shot Target
        if(DriverStation.getAlliance().isEmpty()){
            shotState = Optional.empty();
            return;
        } 
        var shotTarget = DriverStation.getAlliance().get() == Alliance.Red ? Vision.RED_SHOT_TARGET : Vision.BLUE_SHOT_TARGET;
    
        //Range To Target
        var optionalRange = VisionSubsystem.getTargetDistance();
        if(optionalRange.isEmpty()){
            shotState = Optional.empty();
            return;
        } 
        var rangeToTarget = optionalRange.getAsDouble();

        //Robot Pose
        var robotPose =  this.driveSubsystem.getRobotPose2d();

        //Robot Velocity
        var robotVelocity = this.driveSubsystem.getChassisSpeed();

        //dt (Todo: find actual dt)
        var dt = 0.3;

        this.shotState =  Optional.of(ShotState.computedFromPose(
            shotTarget,
            rangeToTarget,
            robotPose,
            robotVelocity,
            dt
        ));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("April Tag Id", () -> _limelightCapture.aprilTagId() , null);
        builder.addBooleanProperty("LL Found Target", () -> _limelightCapture.hasTarget(), null);
        builder.addDoubleProperty("LL Target xOffset", () -> _limelightCapture.xOffset(), null);
        builder.addDoubleProperty("LL Target yOffset",  () -> _limelightCapture.yOffset(), null);
        builder.addDoubleArrayProperty("LL BotPose TargetSpace",  () -> _limelightCapture.robotpose_targetspace(), null);
        builder.addDoubleArrayProperty("LL CameraPose TargetSpace",  () -> _limelightCapture.camerapose_targetspace(), null);
        builder.addDoubleArrayProperty("LL TargetPose RobotSpace",  () -> _limelightCapture.targetpose_robotspace(), null);
        builder.addDoubleArrayProperty("LL TargetPose CameraSpace",  () -> _limelightCapture.targetpose_cameraspace(), null);
        builder.addDoubleProperty("Target Distance", () -> VisionSubsystem.getTargetDistance().orElse(-1.0),null);
        builder.addDoubleProperty("ShotState Heading", () -> shotState.isPresent() ? shotState.get().get_heading().getDegrees() : Double.NaN, null);
    }
}
