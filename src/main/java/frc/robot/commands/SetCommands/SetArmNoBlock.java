package frc.robot.commands.SetCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmNoBlock extends Command{
    DoubleSupplier target;
    ArmSubsystem arm;
    public SetArmNoBlock(ArmSubsystem arm, double radTarget) {
        this(arm,()->radTarget);
    }

    public SetArmNoBlock(ArmSubsystem arm, DoubleSupplier radTarget) {
        addRequirements(arm);
        this.arm = arm;
        target = radTarget;
    }

    @Override
    public void initialize() {
        arm.setTargetRads(target.getAsDouble());
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
