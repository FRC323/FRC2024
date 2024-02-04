package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetFeederSpeed extends Command {

    private  double feederVel = 0;
    private ArmSubsystem armSubsystem;

    public SetFeederSpeed(ArmSubsystem arm, double feederVel) {
        this.feederVel = feederVel;
        this.armSubsystem = arm;
    }
    @Override
    public void execute(){
        armSubsystem.setFeederSpeed(this.feederVel);
    }
//    @Override
//    public boolean isFinished() {
//        return true;
//    }
}
