package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmControl extends Command{

    private ArmSubsystem armSubsystem;
    private boolean direction;

    public ManualArmControl(ArmSubsystem armSubsystem,boolean direction){
        this.armSubsystem = armSubsystem;
        this.direction = direction;        
    }

    @Override
    public void execute(){
        armSubsystem.setArmPower(direction ? -0.2 : 0.2);
    }

    @Override
    public void end(boolean interupted){
        armSubsystem.setArmPower(0.0);
        armSubsystem.setTargetRads(armSubsystem.getArmAngleRads());
    }
}
