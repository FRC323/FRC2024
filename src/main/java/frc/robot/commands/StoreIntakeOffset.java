package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StoreIntakeOffset extends Command {
  private IntakeSubsystem intakeSubsystem;

  public StoreIntakeOffset(IntakeSubsystem intakeSubsystem) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void execute() {
    this.intakeSubsystem.storeWristOffset();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
