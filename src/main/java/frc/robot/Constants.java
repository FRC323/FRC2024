// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
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
    public static final double MAX_SPEED_METERS_PER_SECOND = 22.3,
        MAX_ANGULAR_SPEED_RAD_PER_SECONDS = 4 * 2 * Math.PI; // radians per second

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
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_2 = 2.0;
    public static final double MAX_ANGULAR_ACCELERATION_RADS_PER_SECOND_2 = 2.0;

    public static int FRONT_LEFT_DRIVING_CAN_ID = 11;
    public static int FRONT_LEFT_TURNING_CAN_ID = 21;
    public static boolean FRONT_LEFT_IS_INVERTED = false;
    public static double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD = 0.0;
    public static String FRONT_LEFT_OFFSET_KEY = "FL_Offset";

    public static int FRONT_RIGHT_DRIVING_CAN_ID = 12;
    public static int FRONT_RIGHT_TURNING_CAN_ID = 22;
    public static double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD = 0.0;
    public static boolean FRONT_RIGHT_IS_INVERTED = true;
    public static String FRONT_RIGHT_OFFSET_KEY = "FR_Offset";

    public static int REAR_LEFT_DRIVING_CAN_ID = 14;
    public static int REAR_LEFT_TURNING_CAN_ID = 24;
    public static double REAR_LEFT_CHASSIS_ANGULAR_OFFSET_RAD = 0.0;
    public static boolean REAR_LEFT_IS_INVERTED = false;
    public static String REAR_LEFT_OFFSET_KEY = "RL_Offset";

    public static int REAR_RIGHT_DRIVING_CAN_ID = 13;
    public static int REAR_RIGHT_TURNING_CAN_ID = 23;
    public static double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD = 0.0;
    public static boolean REAR_RIGHT_IS_INVERTED = true;
    public static String REAR_RIGHT_OFFSET_KEY = "RR_Offset";

    public static class Module {
      //      TODO: Group these in a sane way
      public static final boolean DRIVING_ENCODER_INVERTED = true;
      public static final double DRIVING_K_P = 0.1;
      public static final double DRIVING_K_I = 0.0;
      public static final double DRIVING_K_D = 0.1;
      public static final double DRIVING_K_FF =
          0.95 / Constants.Swerve.Module.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;
      public static final double DRIVING_MIN_OUTPUT = -1.0;
      public static final double DRIVING_MAX_OUTPUT = 1.0;
      public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 40;
      public static final double DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR = 1.0;
      public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotor.FREE_SPEED_RPM / 60;

      public static final CANSparkBase.IdleMode DRIVING_MOTOR_IDLE_MODE =
          CANSparkBase.IdleMode.kCoast;

      /** Multiplier for wheel diameter based on empirical on-field measurement */
      public static final double WHEEL_DIAMETER_FUDGE_FACTOR = 1.0;

      public static final double WHEEL_DIAMETER_METERS =
          Units.inchesToMeters(3) * WHEEL_DIAMETER_FUDGE_FACTOR;
      private static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
      //      12T	 5.50:1
      //      13T	 5.08:1
      //      14T	 4.71:1
      private static final double DRIVING_MOTOR_REDUCTION = 4.71;
      public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS =
          WHEEL_CIRCUMFERENCE_METERS / DRIVING_MOTOR_REDUCTION; // meters
      public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND =
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
    }
  }

  public static class PathFollowing{
    public static final PIDConstants DRIVE_PID_CONSTANTS = new PIDConstants(6.0,0.0,0.0); 
    public static final PIDConstants STEER_PID_CONSTANTS = new PIDConstants(8.0,0.0,0.0);

    public static final HolonomicPathFollowerConfig holonomicPathFollowerConfig =
      new HolonomicPathFollowerConfig(
          Constants.PathFollowing.DRIVE_PID_CONSTANTS,
          Constants.PathFollowing.STEER_PID_CONSTANTS,
          Constants.Swerve.MAX_SPEED_METERS_PER_SECOND,
          Constants.Swerve.WHEEL_BASE_METERS,
          new ReplanningConfig());
  }

  public static class Arm {
    // CAN IDs
    public static final int Arm_Actuation_L = 41;
    public static final int Arm_Actuation_R = 42;

    // Profiled PID Constants //TODO:Tune values
    public static final double kP = 11.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final int ENCODER_PORT = 0;
    public static final int BEAM_BREAK_PORT = 2;
    public static final String OFFSET_KEY = "Arm_Offset";
    public static final double MAX_VELOCITY = Units.degreesToRadians(240);
    public static final double MAX_ACCELERATION = Units.degreesToRadians(1920);

    public static final TrapezoidProfile.Constraints ARM_CONSTRAINTS =
        new Constraints(MAX_VELOCITY, MAX_ACCELERATION);

    // Feedforward
    public static final double kG = 0.36; // Volts
    public static final double kV = 3.12; // Volts * s/rad
    public static final double kA = 0.02; // Volts * s^2/rad

    // Motor Constants
    public static final int CURRENT_LIMIT = 40;

    public static final double SOFT_LIMIT_MIN = -2.08;
    public static final double SOFT_LIMIT_MAX = Units.degreesToRadians(0.0);

    public static class Shooter {
      public static final int Shooter_L_CAN_Id = 61;
      public static final int Shooter_R_CAN_Id = 62;

      public static final int Feeder_CAN_Id = 51;

      public static final double MAX_SHOOTER_VELOCITY = 100024;
      public static final double MAX_SHOOTER_ACCELERATION = 6012;

      public static final TrapezoidProfile.Constraints CONSTRAINTS =
          new Constraints(MAX_SHOOTER_VELOCITY, MAX_SHOOTER_ACCELERATION);

      public static final double kP = 0.1;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double SPEAKER_SPEED = -1.0;
      public static final double AMP_SPEED = -0.75;
    }
    public static final double ARM_INTAKE_UNFOLDING_POSE = -0.8;
    public static final double ARM_DOWN_POSE = 0;
    public static final double ARM_HANDOFF_POSE = -0.25; 
    public static final double ARM_AMP_POSE = -2.0;
    public static final double ARM_HUMAN_PLAYER_POSE =  -1.3;
    public static final double ARM_FAR_SPEAKER = -0.6;

    public static final double FEEDER_INTAKE_SPEED = -0.75;
    public static final double FEED_SHOOT_SPEED = -1.0;
    
  }

  public static class Intake {
    // CAN IDs
    public static final int ROLLER_ID = 32;
    public static final int ACTUATION_ID = 31;

    // Profiled PID Constants //TODO:Tune values
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final int ENCODER_PORT = 1;
    public static final String OFFSET_KEY = "Wrist_Offset";
    public static final double MAX_VELOCITY = Units.degreesToRadians(360.0);
    public static final double MAX_ACCELERATION = Units.degreesToRadians(4096);

    public static final TrapezoidProfile.Constraints WRIST_CONSTRAINTS =
        new Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    
      public static final int CURRENT_LIMIT = 30;

    //TODO: Get actual constants
    public static final double ENCODER_GEAR_RATION = (16.0/22.0);
    public static final double SOFT_LIMIT_MIN = Units.degreesToRadians(0.0);
    public static final double SOFT_LIMIT_MAX = Units.degreesToRadians(180);

    public static final double UNFOLDED_POSE = 3.1;
    public static final double FOLDED_POSE = 0.1; 
    
    public static final double INTAKE_SPEED = 0.5;
    public static final double OUTTAKE_SPEED = -0.5;

    public static final double kS = 0.0;
    public static double kV = 0.0;//0.1;//0.73;
     public static double kA =  0.01;
//    TODO: We should check this
     public static double kG = 0.03; //0.1 //0.57; //0.25
  }
}
