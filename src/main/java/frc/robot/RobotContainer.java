// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandJoystick m_driveJoystick = new CommandJoystick(0);
  private final CommandJoystick m_steerJoystick = new CommandJoystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    Shuffleboard.getTab("Subsystems").add(driveSubsystem.getName(), driveSubsystem);
    Shuffleboard.getTab("Subsystems").add(armSubsystem.getName(), armSubsystem);
    Shuffleboard.getTab("Subsystems").add(intakeSubsystem.getName(), intakeSubsystem);
    SmartDashboard.putData(driveSubsystem);
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

    //    TODO: Field Centric Enable
    driveSubsystem.setDefaultCommand(
        new RunCommand(
            () ->
                driveSubsystem.drive(
                    m_driveJoystick.getY(), m_driveJoystick.getX(), m_steerJoystick.getX(), false),
            driveSubsystem));

    SmartDashboard.putData(new StoredDrivetrainOffsets(driveSubsystem));
    SmartDashboard.putData(new StoreArmOffset(armSubsystem));
    SmartDashboard.putData(
        "Arm to Zero", new SetArmTarget(armSubsystem, Units.degreesToRadians(0)));
    SmartDashboard.putData(
        "Arm to Pickup", new SetArmTarget(armSubsystem, Units.degreesToRadians(17)));
    SmartDashboard.putData(
        "Arm to 60 deg", new SetArmTarget(armSubsystem, Units.degreesToRadians(60)));

    SmartDashboard.putData(
        "Arm to Amp", new SetArmTarget(armSubsystem, Units.degreesToRadians(105)));

    SmartDashboard.putData("Shooter On", new SetShooterSpeed(armSubsystem, -1));
    SmartDashboard.putData("Shooter Slow", new SetShooterSpeed(armSubsystem, -.2));
    SmartDashboard.putData("Shooter Off", new SetShooterSpeed(armSubsystem, 0));
    SmartDashboard.putData("Feeder On", new SetFeederSpeed(armSubsystem, -1));
    SmartDashboard.putData("Feeder Off", new SetFeederSpeed(armSubsystem, 0));
    SmartDashboard.putData("Intake On", new SetIntakeSpeed(intakeSubsystem, -1));
    SmartDashboard.putData("Intake Off", new SetIntakeSpeed(intakeSubsystem, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //    return Autos.exampleAuto(m_exampleSubsystem);
    return new InstantCommand();
  }

  public void simulationInit() {}
}
