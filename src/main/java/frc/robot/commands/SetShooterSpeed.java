package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetShooterSpeed extends Command {

    private DoubleSupplier shooterVel = () -> 0;
    private ArmSubsystem armSubsystem;

    public SetShooterSpeed(ArmSubsystem arm, double shooterVel) {
        this.shooterVel = () -> shooterVel;
        this.armSubsystem = arm;
    }

    public SetShooterSpeed(ArmSubsystem arm, DoubleSupplier shooterVel){
        this.shooterVel = shooterVel;
        this.armSubsystem = arm;
    }

    @Override
    public void execute(){
        armSubsystem.setShooterSpeed(shooterVel.getAsDouble());
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
