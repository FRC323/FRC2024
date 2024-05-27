package frc.robot.commands.SetCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class SetFeederPosition extends Command{
    private FeederSubsystem feederSubsystem;
    private DoubleSupplier positionSupplier;

    public SetFeederPosition(FeederSubsystem feederSubsystem, DoubleSupplier positionSupplier){
        this.feederSubsystem = feederSubsystem;
        this.positionSupplier = positionSupplier;
    }

    public SetFeederPosition(FeederSubsystem feederSubsystem, double position){
        this.feederSubsystem = feederSubsystem;
        this.positionSupplier = () -> position;
    }

    @Override
    public void initialize(){
        this.feederSubsystem.setFeederPoseTarget(this.positionSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return feederSubsystem.atFeederPosition();
    }
}
