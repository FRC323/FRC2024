package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmTarget extends Command {
    double target;
    ArmSubsystem arm;
    public SetArmTarget(ArmSubsystem arm, double radTarget) {
        addRequirements(arm);
        this.arm = arm;
        target = radTarget;
    }

    @Override
    public void execute() {
        arm.setTargetRads(target);
    }

    @Override
    public boolean isFinished() {
        return arm.armIsAtTarget();
    }
}
