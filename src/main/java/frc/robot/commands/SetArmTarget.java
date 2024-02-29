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
    public void end(boolean interupted){
        if(interupted){
            arm.setTargetRads(arm.getArmAngleRads());
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }    

}
