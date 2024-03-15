package frc.robot.commands.SetCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterSpeed extends Command {

    private DoubleSupplier shooterVel = () -> 0;
    private ShooterSubsystem shooterSubsystem;

    public SetShooterSpeed(ShooterSubsystem shooterSubsystem, double shooterVel) {
        this.shooterVel = () -> shooterVel;
        this.shooterSubsystem = shooterSubsystem;
    }

    public SetShooterSpeed(ShooterSubsystem shooterSubsystem, DoubleSupplier shooterVel){
        this.shooterVel = shooterVel;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute(){
        shooterSubsystem.setShooterSpeed(shooterVel.getAsDouble());
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
