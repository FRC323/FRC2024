package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmControl extends Command{

    private ArmSubsystem armSubsystem;
    private boolean direction;
    private double angle = 0.0;

    public ManualArmControl(ArmSubsystem armSubsystem,boolean direction){
        this.armSubsystem = armSubsystem;
        this.direction = direction;        
    }

    @Override
    public void execute(){
        armSubsystem.setTargetRads(
            armSubsystem.getArmAngleRads() + (direction ? -0.01 : 0.01)
        );
        
    }

}
