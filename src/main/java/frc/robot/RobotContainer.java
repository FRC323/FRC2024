// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.AprilTags.Amp;
import frc.robot.Constants.DriverConstants.DriveStick;
import frc.robot.Constants.DriverConstants.SteerStick;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.EjectNote;
import frc.robot.commands.AutoCommands.ResetOdomFromLimelight;
import frc.robot.commands.AutoCommands.ShootAuto;
import frc.robot.commands.ButtonCommands.ClimbCommand;
import frc.robot.commands.ButtonCommands.GotoAmpPose;
import frc.robot.commands.ButtonCommands.IntakeNote;
import frc.robot.commands.ButtonCommands.HumanPlayerPickup;
import frc.robot.commands.ButtonCommands.ManualArmControl;
import frc.robot.commands.ButtonCommands.ManualIntakeControl;
import frc.robot.commands.ButtonCommands.OuttakeCommand;
import frc.robot.commands.ButtonCommands.SafeShotCommand;
import frc.robot.commands.ButtonCommands.ShootCommand;
import frc.robot.commands.Procedures.AlignArmForShot;
import frc.robot.commands.Procedures.AlignWhileDriving;
import frc.robot.commands.Procedures.SetIntakeFoldedInternal;
import frc.robot.commands.Procedures.SetIntakeUnfolded;
import frc.robot.commands.Procedures.SetIntakeUp;
import frc.robot.commands.SetCommands.SetArmTarget;
import frc.robot.commands.SetCommands.SetFeederSpeed;
import frc.robot.commands.SetCommands.SetIntakeSpeed;
import frc.robot.commands.SetCommands.SetIntakeTarget;
import frc.robot.commands.SetCommands.SetShooterSpeed;
import frc.robot.commands.StoreOffsetCommands.StoreArmOffset;
import frc.robot.commands.StoreOffsetCommands.StoreIntakeOffset;
import frc.robot.commands.StoreOffsetCommands.StoreDrivetrainOffsets;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final ArmSubsystem armSubsystem = new ArmSubsystem();
  public final FeederSubsystem feederSubsystem = new FeederSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final CommandJoystick m_driveJoystick =
      new CommandJoystick(Constants.DriverConstants.kDriveStickPort);
  private final CommandJoystick m_steerJoystick =
      new CommandJoystick(Constants.DriverConstants.kSteerStickPort);

  private final SendableChooser<Command> autoChooser;

  private IntSupplier invertedDriveStick =
      () -> DriverStation.getAlliance().get() == Alliance.Red ? 1 : -1;

  private AlignWhileDriving alignWhileDriving =
      new AlignWhileDriving(
          driveSubsystem,
          () -> invertedDriveStick.getAsInt() * m_driveJoystick.getY(),
          () -> invertedDriveStick.getAsInt() * m_driveJoystick.getX(),
          () -> Math.pow(m_steerJoystick.getX(), 2) * Math.signum(-m_steerJoystick.getX()));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    addCommandsToAutoChooser();

    // Configure the trigger bindings
    configureBindings();
    Shuffleboard.getTab("Subsystems").add(driveSubsystem.getName(), driveSubsystem);
    Shuffleboard.getTab("Subsystems").add(armSubsystem.getName(), armSubsystem);
    Shuffleboard.getTab("Subsystems").add(feederSubsystem.getName(), feederSubsystem);
    Shuffleboard.getTab("Subsystems").add(shooterSubsystem.getName(), shooterSubsystem);
    Shuffleboard.getTab("Subsystems").add(intakeSubsystem.getName(), intakeSubsystem);
    Shuffleboard.getTab("Field").add(driveSubsystem.getField());

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

    // Drive Stick
    driveSubsystem.setDefaultCommand(
        new RunCommand(
            () ->
                driveSubsystem.drive(
                    invertedDriveStick.getAsInt() * m_driveJoystick.getY(),
                    invertedDriveStick.getAsInt() * m_driveJoystick.getX(),
                    Math.pow(m_steerJoystick.getX(), 2) * Math.signum(-m_steerJoystick.getX()) * (m_steerJoystick.getHID().getTrigger() ? 0.0 : 1.0),
                    true) // !m_steerJoystick.trigger().getAsBoolean())
            ,
            driveSubsystem));


    shooterSubsystem.setDefaultCommand(
        new SetShooterSpeed(shooterSubsystem, 0)
    );

    feederSubsystem.setDefaultCommand(
        new SetFeederSpeed(feederSubsystem, 0)
    );

    // intakeSubsystem.setDefaultCommand(
    //     new SetIntakeSpeed(intakeSubsystem, 0)
    // );

    //TODO make this less janky
    intakeSubsystem.setDefaultCommand(
        new SequentialCommandGroup(
            new SetIntakeSpeed(intakeSubsystem, 0),
            new ConditionalCommand(
                    new ConditionalCommand(
                        new SetIntakeUp(armSubsystem, intakeSubsystem),
                        new InstantCommand(),
                        () -> intakeSubsystem.getWristAngleRads() > Intake.FOLDED_POSE_INTERNAL + Constants.MARGIN_OF_ERROR_RADS // TO check if robot is in folded pose
                    ),
                new InstantCommand(),
                () -> shooterSubsystem.getCurrentCommand() == null // This check is here to make sure the robot doesn't fold up when shooting
            )
        )
    );


    // Reset Gyro
    m_driveJoystick
        .button(DriveStick.RIGHT_SIDE_BUTTON)
        .onTrue(
            new InstantCommand(
                () ->
                    driveSubsystem.resetYawToAngle(
                        invertedDriveStick.getAsInt() == 1 ? 180.0 : 0.0),
                driveSubsystem));

    // Align While Driving
    m_steerJoystick
        .trigger()
        .whileTrue(
            new ParallelCommandGroup(
                alignWhileDriving,
                new AlignArmForShot(armSubsystem, shooterSubsystem, feederSubsystem, intakeSubsystem, driveSubsystem)
            )
        );

    // // Shoot
    m_driveJoystick
        .button(DriveStick.LEFT_SIDE_BUTTON)
        .toggleOnTrue(
            new ShootCommand(feederSubsystem, shooterSubsystem, armSubsystem)
        );
    // // Intake Button
    m_driveJoystick
        .trigger()
        // This somehow breaks transitions to other commands ughh
        //.whileFalse(new SetIntakeSpeed(intakeSubsystem, 0))
        .whileTrue(
            new IntakeNote(intakeSubsystem, armSubsystem, feederSubsystem)
        );

    // Outtake
    m_driveJoystick
        .button(DriveStick.TOP_BIG_BUTTON)
        // This somehow breaks transitions to other commands ughh
        //.whileFalse(new SetIntakeSpeed(intakeSubsystem, 0))
        .whileTrue(
            new OuttakeCommand(armSubsystem, intakeSubsystem, feederSubsystem, shooterSubsystem)
        );

    // Folded 
    m_driveJoystick
        .button(Constants.DriverConstants.DriveStick.BACK_SIDE_BUTTON)
        .whileTrue(
            new SetIntakeFoldedInternal(intakeSubsystem, armSubsystem, feederSubsystem)
        );

    // //Human Player Pickup 
    m_steerJoystick
        .button(SteerStick.MIDDLE)
        .whileTrue(new HumanPlayerPickup(intakeSubsystem, armSubsystem, feederSubsystem));

    // Safe Zone Shot
    m_steerJoystick
        .button(SteerStick.RIGHT)
        .onTrue(new SafeShotCommand(intakeSubsystem, armSubsystem, shooterSubsystem, feederSubsystem));

    // // Amp Pose
    m_steerJoystick
        .button(SteerStick.LEFT)
        .onTrue(
            new GotoAmpPose(armSubsystem, intakeSubsystem, shooterSubsystem, feederSubsystem)
        );

    // // Climb Button
    m_driveJoystick
        .button(DriveStick.SMALL_TOP_BUTTON)
        .whileTrue(
            new ClimbCommand(armSubsystem, intakeSubsystem)
        );

    // // Manual Arm
    m_driveJoystick
        .button(DriveStick.UP_DIRECTIONAL)
        .whileTrue(
            new SequentialCommandGroup(
                new ManualArmControl(armSubsystem, -0.1)));
    m_driveJoystick
        .button(DriveStick.DOWN_DIRECTIONAL)
        .whileTrue(
            new SequentialCommandGroup(
                new ManualArmControl(armSubsystem, 0.1)));

    m_driveJoystick
        .button(DriveStick.LEFT_DIRECTIONAL)
        .whileTrue(
            new SequentialCommandGroup(
                new ManualArmControl(armSubsystem, -0.02)));
    m_driveJoystick
        .button(DriveStick.RIGHT_DIRECTIONAL)
        .whileTrue(
            new SequentialCommandGroup(
                new ManualArmControl(armSubsystem, 0.01)));
    //Manual Intake
    m_driveJoystick
        .povUp()
        .whileTrue(
            new ManualIntakeControl(intakeSubsystem, true).handleInterrupt(
           () -> intakeSubsystem.setWristPower(0.0)
            )
        );
    
    m_driveJoystick
        .povDown()
        .whileTrue(
            new ManualIntakeControl(intakeSubsystem, false).handleInterrupt(
           () -> intakeSubsystem.setWristPower(0.0)
            )
        );
    
    // m_driveJoystick
    //     .button(DriveStick.SMALL_TOP_BUTTON)
        // .onTrue(new ResetOdomFromLimelight(poseEstimatorSubsystem));
        
  }

  private void addShuffleBoardData() {
    Shuffleboard.getTab("Buttons").add(new StoreDrivetrainOffsets(driveSubsystem));
    Shuffleboard.getTab("Buttons").add(new StoreArmOffset(armSubsystem));
    Shuffleboard.getTab("Buttons").add(new StoreIntakeOffset(intakeSubsystem));

    Shuffleboard.getTab("Buttons")
        .add(
            "Go 1 Meter",
            PathFollowerCommands.createDriveToAbsolutePositionCommand(
                driveSubsystem, 1, 0.00, 90.0));

    Shuffleboard.getTab("Buttons")
        .add("Follow Path", PathFollowerCommands.followPathFromFile(driveSubsystem, "Test Path"));

    Shuffleboard.getTab("Buttons").add("Shooter Slow", new SetShooterSpeed(shooterSubsystem, -.2));
    Shuffleboard.getTab("Buttons").add("Shooter On", new RepeatCommand(new SetShooterSpeed(shooterSubsystem, Shooter.SHOOTER_SPEED)));
    Shuffleboard.getTab("Buttons").add("Shooter Off", new SetShooterSpeed(shooterSubsystem, 0));
    Shuffleboard.getTab("Buttons").add("Feeder On", new RepeatCommand(new SetFeederSpeed(feederSubsystem, -1.0)));
    Shuffleboard.getTab("Buttons").add("Feeder Off", new SetFeederSpeed(feederSubsystem, 0));
    Shuffleboard.getTab("Buttons").add("Intake On", new SetIntakeSpeed(intakeSubsystem, 0.5));
    Shuffleboard.getTab("Buttons").add("Intake Off", new SetIntakeSpeed(intakeSubsystem, 0));
    Shuffleboard.getTab("Buttons").add("Align While Driving", alignWhileDriving);
    Shuffleboard.getTab("Buttons").add("Feed Note", new SequentialCommandGroup(
        new SetFeederSpeed(feederSubsystem, Constants.Feeder.FEEDER_INTAKE_SPEED),
        new WaitUntilCommand(feederSubsystem::isHoldingNote),
        new SetFeederSpeed(feederSubsystem, 0.0)
    ));

    // Shuffleboard.getTab("Buttons")
        // .add("HandoffProc", new HandoffProc(intakeSubsystem, armSubsystem, feederSubsystem));

    Shuffleboard.getTab("Buttons")
        .add("ResetPose", new ResetOdomFromLimelight(driveSubsystem));

    Shuffleboard.getTab("Buttons").add("Auto Chooser", autoChooser);

    Shuffleboard.getTab("Buttons").add("Disable Encoder Control", new InstantCommand(() -> armSubsystem.disableEncoderControl(true)));
    Shuffleboard.getTab("Buttons").add("Enable Encoder Control", new InstantCommand(() -> armSubsystem.disableEncoderControl(false)));

    // SmartDashboard.putData("Pick Note",new FireNoteAuto(driveSubsystem, intakeSubsystem,
    // armSubsystem));
  }

  private void addCommandsToAutoChooser() {
    NamedCommands.registerCommand("HandoffProc", new IntakeNote(intakeSubsystem, armSubsystem, feederSubsystem));
    NamedCommands.registerCommand("Reset Odom", new ResetOdomFromLimelight(driveSubsystem));
    NamedCommands.registerCommand(
        "UnfoldIntake", new SetIntakeUnfolded(intakeSubsystem, armSubsystem));
    NamedCommands.registerCommand(
        "ShootAuto", new ShootAuto(driveSubsystem, armSubsystem, intakeSubsystem, shooterSubsystem, feederSubsystem));
    NamedCommands.registerCommand("EjectNote", new EjectNote(shooterSubsystem, feederSubsystem, intakeSubsystem, armSubsystem));
    NamedCommands.registerCommand("AlignArm", new AlignArmForShot(armSubsystem, shooterSubsystem, feederSubsystem, intakeSubsystem, driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //    return Autos.exampleAuto(m_exampleSubsystem);
    // return new SequentialCommandGroup(
    // new ResetOdomFromLimelight(poseEstimatorSubsystem),
    return autoChooser.getSelected();
    // );

  }

  public void simulationInit() {}
}
