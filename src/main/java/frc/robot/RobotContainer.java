// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriverConstants.DriveStick;
import frc.robot.Constants.DriverConstants.SteerStick;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.ResetOdomFromLimelight;
import frc.robot.commands.AutoCommands.ShootAuto;
import frc.robot.commands.AutoCommands.StartShooterWheelSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(driveSubsystem, visionSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandJoystick m_driveJoystick = new CommandJoystick(Constants.DriverConstants.kDriveStickPort);
  private final CommandJoystick m_steerJoystick = new CommandJoystick(Constants.DriverConstants.kSteerStickPort);

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    addCommandsToAutoChooser();

    // Configure the trigger bindings
    configureBindings();
    Shuffleboard.getTab("Subsystems").add(driveSubsystem.getName(), driveSubsystem);
    Shuffleboard.getTab("Subsystems").add(armSubsystem.getName(), armSubsystem);
    Shuffleboard.getTab("Subsystems").add(intakeSubsystem.getName(), intakeSubsystem);
    Shuffleboard.getTab("Subsystems").add(visionSubsystem.getName(), visionSubsystem);
    Shuffleboard.getTab("Subsystems").add(poseEstimatorSubsystem.getName(),poseEstimatorSubsystem);
    SmartDashboard.putData(driveSubsystem);

    autoChooser = AutoBuilder.buildAutoChooser();    
    addShuffleBoardData();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    //Drive Stick
    driveSubsystem.setDefaultCommand(
        new RunCommand(
            () ->
                driveSubsystem.drive(
                    -m_driveJoystick.getY(),
                    -m_driveJoystick.getX(),
                    Math.pow(m_steerJoystick.getX(),2) * Math.signum(-m_steerJoystick.getX()),
                    true),// !m_steerJoystick.trigger().getAsBoolean()),
            driveSubsystem));

    m_steerJoystick.trigger().whileTrue(
      new AlignToTargetXOffset(
        visionSubsystem, driveSubsystem, m_driveJoystick, Constants.AprilTags.Speaker.HEIGHT, Constants.AprilTags.Speaker.TAGS_CENTER)
      );
    //Reset Gyro
    m_driveJoystick.button(DriveStick.RIGHT_SIDE_BUTTON).onTrue(
      new InstantCommand(
        () ->
          driveSubsystem.resetYaw()
        , driveSubsystem)
    );

    //Handoff Button
    m_driveJoystick.trigger().whileTrue(
      new HandoffProc(intakeSubsystem, armSubsystem).handleInterrupt(
        ()->{
          intakeSubsystem.setIntakeSpeed(0);
          armSubsystem.setFeederSpeed(0);
        }
      )
    );

    //Outtake
    m_driveJoystick.button(DriveStick.TOP_BIG_BUTTON).whileTrue(
      new SequentialCommandGroup(
        new SetArmTarget(armSubsystem, Constants.Arm.ARM_INTAKE_UNFOLDING_POSE),
        new SetIntakeSpeed(intakeSubsystem, Constants.Intake.OUTTAKE_SPEED),
        new ConditionalCommand(
          new SetFeederSpeed(armSubsystem, Constants.Arm.FEEDER_REVERSE_SPEED),
          new InstantCommand(),
          ()->armSubsystem.getArmAngleRads() <= Constants.Arm.ARM_HANDOFF_POSE
        ),
        new SetFeederSpeed(armSubsystem, Constants.Arm.FEEDER_REVERSE_SPEED),
        new SetShooterSpeed(armSubsystem, Constants.Arm.Shooter.REVERSE_SPEED)
      )
    ).onFalse(
        new ParallelCommandGroup(
          new SetIntakeSpeed(intakeSubsystem, 0),
          new SetFeederSpeed(armSubsystem, 0),
          new SetShooterSpeed(armSubsystem, 0)
        )
    );

    //Shoot
    m_driveJoystick.button(DriveStick.LEFT_SIDE_BUTTON).whileTrue(
      new ShootCommand(armSubsystem,Constants.Arm.Shooter.SPEAKER_SPEED).handleInterrupt(
        () -> {
          armSubsystem.setFeederSpeed(0);
          armSubsystem.setShooterSpeed(0);
        }
      )
    );

    //Folded (Must be Held)
    m_driveJoystick.button(DriveStick.BACK_SIDE_BUTTON).whileTrue(
      new SetIntakeFolded(intakeSubsystem,armSubsystem).handleInterrupt(
        () -> {
          armSubsystem.setTargetRads(armSubsystem.getArmAngleRads());
          intakeSubsystem.setTargetRads(intakeSubsystem.getWristAngleRads());
        }
      )
    );
    
    //Arm Poses
    m_steerJoystick.button(SteerStick.MIDDLE).whileTrue(
        new HumanPlayerPickup(intakeSubsystem,armSubsystem)
    ).onFalse(
      new SetFeederSpeed(armSubsystem, 0)
    );

    m_steerJoystick.button(SteerStick.LEFT).onTrue(
      new SetArmTarget(armSubsystem, Constants.Arm.ARM_AMP_POSE)
    );

    m_steerJoystick.button(SteerStick.RIGHT).onTrue(
      new SetArmTarget(armSubsystem,Constants.Arm.ARM_FAR_SPEAKER)
    );

    //Manual Arm
    m_driveJoystick.button(DriveStick.UP_DIRECTIONAL).whileTrue(
      new ManualArmControl(armSubsystem,true)
    );
    m_driveJoystick.button(DriveStick.DOWN_DIRECTIONAL).whileTrue(
      new ManualArmControl(armSubsystem,false)
    );


  }

  private void addShuffleBoardData(){
    SmartDashboard.putData(new StoredDrivetrainOffsets(driveSubsystem));
    SmartDashboard.putData(new StoreArmOffset(armSubsystem));
    SmartDashboard.putData(new StoreIntakeOffset(intakeSubsystem));

    // SmartDashboard.putData(
    //     "Arm to Down", new SetArmTarget(armSubsystem, Constants.Arm.ARM_DOWN_POSE));
    // SmartDashboard.putData(
    //     "Arm to Handoff", new SetArmTarget(armSubsystem, Constants.Arm.ARM_HANDOFF_POSE));
    // SmartDashboard.putData(
    //     "Arm to Min Unfolded", new SetArmTarget(armSubsystem, Constants.Arm.ARM_INTAKE_UNFOLDING_POSE));

    // SmartDashboard.putData(
        // "Arm to Amp", new SetArmTarget(armSubsystem, Units.degreesToRadians(105)));

    // SmartDashboard.putData(
    //   "Intake to Unfolded", new SetIntakeTarget(intakeSubsystem, Constants.Intake.UNFOLDED_POSE)
    // );
    // SmartDashboard.putData(
    //   "Intake to Folded", new SetIntakeTarget(intakeSubsystem, Constants.Intake.FOLDED_POSE)
    // );
    
    //april tag testing
    // SmartDashboard.putData(
    //   "Align to Speaker", new AlignToTarget(visionSubsystem, driveSubsystem, Constants.AprilTags.Speaker.HEIGHT, Constants.AprilTags.Speaker.TAGS_CENTER)
    // );

    // SmartDashboard.putData(
    //   "Align to Amp", new AlignToTarget(visionSubsystem, driveSubsystem, Constants.AprilTags.Amp.HEIGHT, Constants.AprilTags.Amp.TAGS)
    // );

    SmartDashboard.putData(
      "Folded Command", new SetIntakeFolded(intakeSubsystem,armSubsystem)
    );
    SmartDashboard.putData(
      "Unfolded Command", new SetIntakeUnfolded(intakeSubsystem,armSubsystem)
    );

    SmartDashboard.putData(
      "Go 1 Meter", PathFollowerCommands.createDriveToAbsolutePositionCommand(driveSubsystem, 1, 0.00, 90.0)
    );

    SmartDashboard.putData(
      "Follow Path", PathFollowerCommands.followPathFromFile(driveSubsystem, "Test Path")
    );

    SmartDashboard.putData("Shooter On", new SetShooterSpeed(armSubsystem, -1));
    SmartDashboard.putData("Shooter Slow", new SetShooterSpeed(armSubsystem, -.2));
    SmartDashboard.putData("Shooter Off", new SetShooterSpeed(armSubsystem, 0));
    SmartDashboard.putData("Feeder On", new SetFeederSpeed(armSubsystem, -1.0));
    SmartDashboard.putData("Feeder Off", new SetFeederSpeed(armSubsystem, 0));
    SmartDashboard.putData("Intake On", new SetIntakeSpeed(intakeSubsystem, 0.5));
    SmartDashboard.putData("Intake Off", new SetIntakeSpeed(intakeSubsystem, 0));

    SmartDashboard.putData("HandoffProc",new HandoffProc(intakeSubsystem, armSubsystem));

    SmartDashboard.putData("ResetPose",new InstantCommand(()-> driveSubsystem.resetOdometry(new Pose2d(14.5,4.5,new Rotation2d(Units.degreesToRadians(-150.0)))),driveSubsystem)); 
  
    SmartDashboard.putData("Auto Chooser",autoChooser);

    SmartDashboard.putData("Pick Note",new FireNoteAuto(driveSubsystem, intakeSubsystem, armSubsystem));
  }

  private void addCommandsToAutoChooser(){
    NamedCommands.registerCommand("HandoffProc", new HandoffProc(intakeSubsystem, armSubsystem));
  //   NamedCommands.registerCommand("ShootAmp", new ShootAmp(armSubsystem));
    // NamedCommands.registerCommand("ShootSpeaker", new ShootSpeaker(armSubsystem));
  //   NamedCommands.registerCommand("Fold Intake", new SetIntakeFolded(intakeSubsystem, armSubsystem));
  NamedCommands.registerCommand("StartShooterWheelSpeaker", new InstantCommand(()->{armSubsystem.setShooterSpeed(Constants.Arm.Shooter.SPEAKER_SPEED);},armSubsystem));
  NamedCommands.registerCommand("UnfoldIntake", new SetIntakeUnfolded(intakeSubsystem, armSubsystem));
  NamedCommands.registerCommand("RunIntake",
    new InstantCommand(()->{intakeSubsystem.setIntakeSpeed(Constants.Intake.INTAKE_SPEED);},intakeSubsystem));
  NamedCommands.registerCommand("RunFeeder",
    new InstantCommand(()->{armSubsystem.setFeederSpeed(Constants.Arm.FEEDER_INTAKE_SPEED);},armSubsystem));
  NamedCommands.registerCommand("ShootSpeaker", new ShootAuto(driveSubsystem, armSubsystem, visionSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //    return Autos.exampleAuto(m_exampleSubsystem);
    return new SequentialCommandGroup(
      new ResetOdomFromLimelight(poseEstimatorSubsystem),
      autoChooser.getSelected()
    );
      
  }

  public void simulationInit() {}
}
