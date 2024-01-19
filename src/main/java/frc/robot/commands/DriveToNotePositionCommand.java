package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.List;

public class DriveToNotePositionCommand extends Command {

    /*
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
          new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
          new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
          new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
  );

  // Create the path using the bezier points created above
  PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
          new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  );

     */
    private PathPlannerPath path;
    private FollowPathHolonomic follower;

    /*
    Arguments to this are in absolute positions compared to the pose of the robot on field
     */
    public DriveToNotePositionCommand(DriveSubsystem drive, double x, double y, double heading) {
        addRequirements(drive);
        List<Translation2d> points = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(x, y, Rotation2d.fromDegrees(0))
        );
    path =
        new PathPlannerPath(
            points,
            new PathConstraints(
                Constants.Swerve.MAX_SPEED_METERS_PER_SECOND,
                Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_2,
                Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS,
                Constants.Swerve.MAX_ANGULAR_ACCELERATION_RADS_PER_SECOND_2),
            new GoalEndState(0.0, Rotation2d.fromDegrees(heading)));

//    TODO: Finish the path follower
//    follower = new FollowPathHolonomic(
//            path,
//            drive::getPose,
//            drive::getChassisSpeed,
//            drive::setChassisSpeed,
//
//
//    )
    }

}
