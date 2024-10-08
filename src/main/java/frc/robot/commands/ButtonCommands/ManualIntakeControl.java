package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualIntakeControl extends Command{

    private IntakeSubsystem intakeSubsystem;
    private boolean direction;

    public ManualIntakeControl(IntakeSubsystem intakeSubsystem,boolean direction){
        this.intakeSubsystem = intakeSubsystem;
        this.direction = direction;        
    }

    @Override
    public void execute(){
        // intakeSubsystem
        // .setTargetRads(
        //     intakeSubsystem.getWristAngleRads() + (direction ? -0.20 : 0.20)
        // );
        
        intakeSubsystem.setWristPower(0.5 * (direction ? -1.0 : 1.0));
    }

}
