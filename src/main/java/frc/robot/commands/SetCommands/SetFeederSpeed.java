package frc.robot.commands.SetCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class SetFeederSpeed extends Command {

    private DoubleSupplier feederVel = ()-> 0;
    private FeederSubsystem feederSubsystem;

    public SetFeederSpeed(FeederSubsystem feederSubsystem, double feederVel) {
        this.feederVel = () -> feederVel;
        this.feederSubsystem = feederSubsystem;
        addRequirements(feederSubsystem);
    }

    public SetFeederSpeed(FeederSubsystem feederSubsystem, DoubleSupplier feederVel){
        this.feederVel = feederVel;
        this.feederSubsystem = feederSubsystem;
        addRequirements(feederSubsystem);
    }

    @Override
    public void execute(){
        feederSubsystem.setFeederSpeed(this.feederVel.getAsDouble());
    }
   @Override
   public boolean isFinished() {
       return true;
   }
}
