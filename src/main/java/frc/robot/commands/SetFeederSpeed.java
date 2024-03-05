package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetFeederSpeed extends Command {

    private DoubleSupplier feederVel = ()-> 0;
    private ArmSubsystem armSubsystem;

    public SetFeederSpeed(ArmSubsystem arm, double feederVel) {
        this.feederVel = () -> feederVel;
        this.armSubsystem = arm;
        addRequirements(arm);
    }

    public SetFeederSpeed(ArmSubsystem arm, DoubleSupplier feederVel){
        this.feederVel = feederVel;
        this.armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void execute(){
        armSubsystem.setFeederSpeed(this.feederVel.getAsDouble());
    }
   @Override
   public boolean isFinished() {
       return true;
   }
}
