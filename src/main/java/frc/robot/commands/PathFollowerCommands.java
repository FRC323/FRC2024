package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.List;

public class PathFollowerCommands extends Command {

  private static final HolonomicPathFollowerConfig holonomicPathFollowerConfig =
      new HolonomicPathFollowerConfig(
          // TODO: These constants are 100% wrong
          Constants.PathFollowing.DRIVE_PID_CONSTANTS,
          Constants.PathFollowing.STEER_PID_CONSTANTS,
          Constants.Swerve.MAX_SPEED_METERS_PER_SECOND,
          Constants.Swerve.WHEEL_BASE_METERS,
          new ReplanningConfig());

  /*
  Arguments to this are in absolute positions compared to the pose of the robot on field
   */
  public static FollowPathHolonomic createDriveToAbsolutePositionCommand(
      DriveSubsystem drive, double x, double y, double heading) {
    //    TODO: Check if the rotations on these are correct, it's possible we want to use heading
    List<Translation2d> points =
        PathPlannerPath.bezierFromPoses(
            new Pose2d(drive.getPose().getX(),drive.getPose().getY(),drive.getPose().getRotation()),
            new Pose2d(x, y, Rotation2d.fromDegrees(heading)));
    PathPlannerPath path;
    path =
        new PathPlannerPath(
            points,
            new PathConstraints(
                Constants.Swerve.MAX_SPEED_METERS_PER_SECOND,
                Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_2,
                Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS,
                Constants.Swerve.MAX_ANGULAR_ACCELERATION_RADS_PER_SECOND_2),
            new GoalEndState(0.0, Rotation2d.fromDegrees(heading)));

    return followPath(drive, path);
  }

  //  Allows easier creation of a drive to command from a relative position, takes into account the
  // current pose of the robot
  //    Note, the heading is still absolute
  public static FollowPathHolonomic createDriveToRelativePositionCommand(
      DriveSubsystem drive, double xOffset, double yOffset, double targetHeading) {
    Translation2d currentTranslation = drive.getPose().getTranslation();

    return createDriveToAbsolutePositionCommand(
        drive,
        currentTranslation.getX() + xOffset,
        currentTranslation.getY() + yOffset,
        targetHeading);
  }

  public static FollowPathHolonomic followPath(DriveSubsystem drive, PathPlannerPath path) {
    return new FollowPathHolonomic(
            path,
            drive::getPose,
            drive::getChassisSpeed,
            drive::setPathFollowerSpeeds,
            holonomicPathFollowerConfig,
            () -> false,
            drive);
  }

  public static FollowPathHolonomic followPathFromFile(DriveSubsystem drive,String path){
    PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(path);
    return followPath(drive, pathPlannerPath);
  }
}
