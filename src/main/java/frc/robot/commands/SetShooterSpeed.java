package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetShooterSpeed extends Command {

    private  double shooterVel = 0;
    private ArmSubsystem armSubsystem;

    public SetShooterSpeed(ArmSubsystem arm, double shooterVel) {
        this.shooterVel = shooterVel;
        this.armSubsystem = arm;
    }
    @Override
    public void execute(){
        armSubsystem.setShooterSpeed(shooterVel);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
