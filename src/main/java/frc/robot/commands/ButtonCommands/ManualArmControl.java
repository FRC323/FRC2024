package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmControl extends Command{

    private ArmSubsystem armSubsystem;
    private double speed;
    private double angle = 0.0;

    public ManualArmControl(ArmSubsystem armSubsystem, double speed){
        this.armSubsystem = armSubsystem;
        this.speed = speed;
    }

    @Override
    public void execute(){
        // armSubsystem.setTargetRads(
        //     armSubsystem.getArmAngleRads() + speed
        // );

        armSubsystem.setArmPower(0.5 * ((speed)/Math.abs(speed)));
        
    }

}
