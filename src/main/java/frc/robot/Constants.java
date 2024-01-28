// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  }

  public static final class UltraPlanetary {
    public static final double REDUCTION_3_1 = 2.89;
    public static final double REDUCTION_4_1 = 3.61;
    public static final double REDUCTION_5_1 = 5.23;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Swerve {

    /**
     * Driving Parameters - Note that these are not the maximum capable speeds of the robot, rather
     * the allowed maximum speeds
     */
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.3,
        MAX_ANGULAR_SPEED_RAD_PER_SECONDS = 2 * Math.PI; // radians per second

    /** Distance between centers of right and left wheels on robot */
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22.5);

    /** Distance between front and back wheels on robot */
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(22.5);

    public static final boolean GYRO_REVERSED = true;

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
      public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
      //      12T	 5.50:1
      //      13T	 5.08:1
      //      14T	 4.71:1
      public static final double DRIVING_MOTOR_REDUCTION = 4.71;
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
      // in KG
      // This mass includes gears and bearings but not fasteners, it's probably close enough
      public static final double WHEEL_MASS = Units.lbsToKilograms(0.6590326);
      //      We're modeling the wheel as a solid cylinder
      public static final double WHEEL_MOI =
          Math.pow(WHEEL_DIAMETER_METERS / 2, 2) * WHEEL_MASS * 0.5;

      // The correct way to  calculate this... missing the last stage reduction
      //      public static final double STEERING_MOTOR_REDUCTION =  UltraPlanetary.REDUCTION_3_1 *
      // UltraPlanetary.REDUCTION_4_1;
      // The wrong way... which was just ask Greg
      public static final double STEERING_MOTOR_REDUCTION = 9424.0/203.0

      public static boolean TURNING_ENCODER_INVERTED = true;
      public static double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0.0;
    }
  }
}
