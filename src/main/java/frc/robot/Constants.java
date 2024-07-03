// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class LED {
    public static final int BLINKIN_PORT = 1;
  }
  
  public static final class AprilTags {
    public static final double APRILTAG_HEIGHT = 9.0;
    public static final double APRILTAG_HEIGHT_HALF = APRILTAG_HEIGHT/2;

    public static final class Speaker {
      public static final int[] TAGS_CENTER = {4,7};
      public static final int[] TAGS_SIDE = {3,8};
      public static final double HEIGHT = 51.96 + APRILTAG_HEIGHT_HALF;
    }

    public static final class Amp {
      public static final int[] TAGS = {5,6};
      public static final double HEIGHT = 48.03 + APRILTAG_HEIGHT_HALF;
    }

    public static final class Source {
      public static final int[] TAGS_LEFT = {2,10};
      public static final int[] TAGS_RIGHT = {1,9};
      public static final double HEIGHT = 48.03 + APRILTAG_HEIGHT_HALF;
    }

    public static final class Stage {
      public static final int[] TAGS = {11,12,13,14,15,16};
      public static final double HEIGHT = 47.63 + APRILTAG_HEIGHT_HALF;
    }
  }

  public static class Vision {
    public static final int APRIL_TAG_PIPELINE = 0;
    public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 25.52;
    public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 18.17;
    public static final Translation2d RED_SHOT_TARGET =  new Translation2d(16.50,5.52);
    public static final Translation2d BLUE_SHOT_TARGET = new Translation2d(0.0,5.52);

    //Todo
    public static final Transform3d BACK_CAMERA_TO_ROBOT = new Transform3d(-0.1661,0.0,0.4616, new Rotation3d(0.0,25.52,0.0));
    public static final Transform3d FRONT_RIGHT_CAMERA_TO_ROBOT = new Transform3d();
    public static final Transform3d FRONT_LEFT_CAMERA_TO_ROBOT = new Transform3d();
  }

  public static final int SPARK_INIT_RETRY_ATTEMPTS = 5;

  public static final class NeoMotor {
    public static final double FREE_SPEED_RPM = 5676;
    public static final double STALL_TORQUE = 3.75; // Theoretical @ 150A
  }

  public static final class UltraPlanetary {
    public static final double REDUCTION_3_1 = 2.89;
    public static final double REDUCTION_4_1 = 3.61;
    public static final double REDUCTION_5_1 = 5.23;
  }


  public static class DriverConstants {
    public static final int kDriveStickPort = 0;
    public static final int kSteerStickPort = 1;

    // Buttons
    public static class DriveStick{
      public static final int LEFT_SIDE_BUTTON = 2;
      public static final int TOP_BIG_BUTTON = 3;
      public static final int BACK_SIDE_BUTTON = 4;
      public static final int UP_DIRECTIONAL = 5;
      public static final int RIGHT_DIRECTIONAL = 6;
      public static final int DOWN_DIRECTIONAL = 7;
      public static final int LEFT_DIRECTIONAL = 8;
      public static final int RIGHT_SIDE_BUTTON = 9;
      public static final int SMALL_TOP_BUTTON = 10;
    }
    public static class SteerStick{
      public static final int LEFT= 2;
      public static final int RIGHT = 3;
      public static final int MIDDLE = 4;
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Swerve {

    /**
     * Driving Parameters - Note that these are not the maximum capable speeds of the robot, rather
     * the allowed maximum speeds
     */
    public static final double MAX_SPEED_METERS_PER_SECOND = 5.66,//4.7,
        MAX_ANGULAR_SPEED_RAD_PER_SECONDS = 3 * 2 * Math.PI; // radians per second

    /** Distance between centers of right and left wheels on robot */
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22.5);

    /** Distance between front and back wheels on robot */
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(22.5);

    public static final boolean GYRO_REVERSED = false;//true;

    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
            new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
            new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
            new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_2 = 20.0; // 20.0
    public static final double MAX_ANGULAR_ACCELERATION_RADS_PER_SECOND_2 = 20.0; // 20.0

    public static int FRONT_LEFT_DRIVING_CAN_ID = 11;
    public static int FRONT_LEFT_TURNING_CAN_ID = 21;
    public static boolean FRONT_LEFT_IS_INVERTED = true;
    public static double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD = 0.0;
    public static String FRONT_LEFT_OFFSET_KEY = "FL_Offset";

    public static int FRONT_RIGHT_DRIVING_CAN_ID = 12;
    public static int FRONT_RIGHT_TURNING_CAN_ID = 22;
    public static double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD = 0.0;
    public static boolean FRONT_RIGHT_IS_INVERTED = false;
    public static String FRONT_RIGHT_OFFSET_KEY = "FR_Offset";

    public static int REAR_LEFT_DRIVING_CAN_ID = 14;
    public static int REAR_LEFT_TURNING_CAN_ID = 24;
    public static double REAR_LEFT_CHASSIS_ANGULAR_OFFSET_RAD = 0.0;
    public static boolean REAR_LEFT_IS_INVERTED = true;
    public static String REAR_LEFT_OFFSET_KEY = "RL_Offset";

    public static int REAR_RIGHT_DRIVING_CAN_ID = 13;
    public static int REAR_RIGHT_TURNING_CAN_ID = 23;
    public static double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD = 0.0;
    public static boolean REAR_RIGHT_IS_INVERTED = false;
    public static String REAR_RIGHT_OFFSET_KEY = "RR_Offset";

    //Align While Driving PID
    public static double ROT_CONTROLLER_KP = 4.5;
    public static double ROT_CONTROLLER_KI = 0;
    public static double ROT_CONTROLLER_KD = 0;
    //public static double ROT_CONTROLLER_FEEDFWD = Math.PI/8;

    public static class Module {
      //      TODO: Group these in a sane way
      public static final boolean DRIVING_ENCODER_INVERTED = true;
      public static final double DRIVING_K_P = 0.2; // 0.1
      public static final double DRIVING_K_I = 0.0;
      public static final double DRIVING_K_D = 0.1; // 0.1
      
      public static final double DRIVING_MIN_OUTPUT = -1.0;
      public static final double DRIVING_MAX_OUTPUT = 1.0;
      public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 40;
      public static final double DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR = 1.0;
      public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotor.FREE_SPEED_RPM / 60;

      public static final CANSparkBase.IdleMode DRIVING_MOTOR_IDLE_MODE =
          CANSparkBase.IdleMode.kCoast;

      /** Multiplier for wheel diameter based on empirical on-field measurement */
      public static final double WHEEL_DIAMETER_FUDGE_FACTOR = 0.978;

      public static final double WHEEL_DIAMETER_METERS =
          Units.inchesToMeters(3) * WHEEL_DIAMETER_FUDGE_FACTOR;
      private static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
      //      12T	 5.50:1
      //      13T	 5.08:1
      //      14T	 4.71:1
      private static final double DRIVING_MOTOR_REDUCTION = 4.00;//4.71;
      public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS =
          WHEEL_CIRCUMFERENCE_METERS / DRIVING_MOTOR_REDUCTION; // meters
      public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND =
        // ((Units.inchesToMeters(3) * Math.PI) / 4.71) / 60.0;    
      DRIVING_ENCODER_POSITION_FACTOR_METERS / 60.0; // meters per second
      public static final double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND =
          DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR
              * ((DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
                  / DRIVING_MOTOR_REDUCTION);
      public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS = 2 * Math.PI;
      public static final double TURNING_K_P = 0.8;
      public static final double TURNING_K_I = 0.0;
      public static final double TURNING_K_D = 0.1;
      public static final double TURNING_K_FF = 0.0;
      public static final double TURNING_MIN_OUTPUT = -1.0;
      public static final double TURNING_MAX_OUTPUT = 1.0;
      public static final CANSparkBase.IdleMode TURNING_MOTOR_IDLE_MODE =
          CANSparkBase.IdleMode.kBrake;
      public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps

      public static boolean TURNING_ENCODER_INVERTED = true;
      public static double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0.0;
      public static final double DRIVING_K_FF =
          0.95 / Constants.Swerve.Module.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;
      
    }
  }

  public static class PathFollowing{
    public static final PIDConstants DRIVE_PID_CONSTANTS = new PIDConstants(6.0,0.0,0.6); 
    public static final PIDConstants STEER_PID_CONSTANTS = new PIDConstants(2.0,0.0,0.0);

    public static final HolonomicPathFollowerConfig holonomicPathFollowerConfig =
      new HolonomicPathFollowerConfig(
          Constants.PathFollowing.DRIVE_PID_CONSTANTS,
          Constants.PathFollowing.STEER_PID_CONSTANTS,
          Constants.Swerve.MAX_SPEED_METERS_PER_SECOND,
          Constants.Swerve.WHEEL_BASE_METERS,
          new ReplanningConfig());
  }

  public static final double MARGIN_OF_ERROR_RADS = 0.15;

  public static class Arm {
    // CAN IDs
    public static final int Arm_Actuation_L = 41;
    public static final int Arm_Actuation_R = 42;

    // Profiled PID Constants 
    public static final double kP = 40.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final int ENCODER_PORT = 0;
    public static final String OFFSET_KEY = "Arm_Offset";
    public static final double OFFSET_FROM_ZERO_POSITION = 0.0;
    public static final double MAX_VELOCITY = Units.degreesToRadians(2048);
    public static final double MAX_ACCELERATION = Units.degreesToRadians(18096);

    public static final TrapezoidProfile.Constraints ARM_CONSTRAINTS =
        new Constraints(MAX_VELOCITY, MAX_ACCELERATION);

    // Feedforward
    public static final double kG = 0.36; // Volts
    public static final double kV = 3.12; // Volts * s/rad
    public static final double kA = 0.02; // Volts * s^2/rad

    // Motor Constants
    public static final int CURRENT_LIMIT = 60;

    public static final double SOFT_LIMIT_MIN = -2.08;
    public static final double SOFT_LIMIT_MAX = Units.degreesToRadians(0.0);
 
    public static final double ARM_INTAKE_UNFOLDING_POSE = -0.9;
    public static final double ARM_DOWN_POSE = 0;
    public static final double ARM_HANDOFF_POSE = -0.25; 
    public static final double ARM_AUTO_STATIC_SHOOT_POSE = -0.40;
    public static final double ARM_OUTAKE_POSE = -0.41;
    public static final double ARM_AMP_POSE = -1.9;
    public static final double ARM_HUMAN_PLAYER_POSE =  -1.27;
    public static final double ARM_SAFE_ZONE_SHOT = -0.695;
    public static final double ARM_CLIMB_POSE = -1.27;

    public static final double AT_TARGET_TOLLERANCE = 0.05;

   
  }

  public static class Feeder{
    public static final int Feeder_CAN_Id = 51;
    public static final int INTIAL_BEAM_BREAK_PORT = 7;

    public static final double FEEDER_INTAKE_SPEED = -0.75;
    public static final double FEED_SHOOT_SPEED = -1.0;
    public static final double FEEDER_REVERSE_SPEED = 0.75;
    public static final double FEEDER_REVERSE_ADJUST = 0.3;
    public static final double FEEDER_ADJUST_SPEED = -0.3;
    public static final double FEEDER_STOPED_SPEED = 0.0;

    public static final double FEEDER_ADJUST_TIME = 0.15;

    public static final double FEEDER_DISTANCE_PER_REV = 2 * Math.PI * 16;

    //TODO: Copied these values from arm. So probally not correct
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double MAX_VELOCITY = Units.degreesToRadians(2048);
    public static final double MAX_ACCELERATION = Units.degreesToRadians(18096);

    public static final TrapezoidProfile.Constraints FEEDER_CONSTRAINTS =
        new Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    public static final double ADJUST_POSITION = 500.0;
    public static final double SHOOT_POSITION = -500.0;
    public static final double POSITION_TOLLERANCE = 2;
 
  }

  public static class Shooter {
      public static final int Shooter_L_CAN_Id = 61;
      public static final int Shooter_R_CAN_Id = 62;

      public static final double kF = 0.0; //* Constants.NeoMotor.FREE_SPEED_RPM;
      public static final double kP = 0.002; //0.0003205128205; //0.0001282051282;
      public static final double kI = 0.000005;
      public static final double kD = 0.0;

      public static final double SHOOTER_SPEED = 5000; //TODO: revert to 5000, low speed b/c small space
      public static final double AMP_SPEED = 2500;
      public static final double REVERSE_SPEED = -400;
      public static final double EJECT_SPEED = 1000;
    }

  public static class Intake {
    // CAN IDs
    public static final int ROLLER_ID = 32;
    public static final int ACTUATION_ID = 31;

    // Profiled PID Constants 
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final int ENCODER_PORT = 1;
    public static final String OFFSET_KEY = "Wrist_Offset";
    public static final double MAX_VELOCITY = Units.degreesToRadians(2048.0);
    public static final double MAX_ACCELERATION = Units.degreesToRadians(4096);

    public static final TrapezoidProfile.Constraints WRIST_CONSTRAINTS =
        new Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    
      public static final int WRIST_CURRENT_LIMIT = 30;
      public static final int ROLLER_CURRENT_LIMIT = 60;

    //TODO: Get actual constants
    public static final double ENCODER_GEAR_RATION = (16.0/22.0);
    public static final double SOFT_LIMIT_MIN = Units.degreesToRadians(0.0);
    public static final double SOFT_LIMIT_MAX = Units.degreesToRadians(180);

    public static final double UNFOLDED_POSE = 3.01;
    public static final double FOLDED_POSE_INTERNAL = 0.02; 
    public static final double SHOOTING_POSE = 1.75;
    public static final double FOLDED_POSE = 0.92;

    //Danger Zones
    // public static final double START_DANGER_ZONE = 2.0;
    // public static final double END_DANGER_ZONE = 2.2;
    
    public static final double INTAKE_SPEED = 0.5;
    public static final double OUTTAKE_SPEED = -0.5;

    public static final double kS = 0.0;
    public static double kV = 0.0;//0.1;//0.73;
     public static double kA =  0.01;
     public static double kG = 0.033; //0.03  //0.1 //0.57; //0.25
  }
}
