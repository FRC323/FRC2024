package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightHelpers;

public class LedSubsystem extends SubsystemBase {
    private final Spark _controller = new Spark(Constants.LED.BLINKIN_PORT);
    private LedColor _color;
    private ArmSubsystem armSubsystem;

    public LedSubsystem(ArmSubsystem armSubsystem) {
        setToAlliance();
        _controller.setSafetyEnabled(false);
        this.armSubsystem = armSubsystem;
    }

    public void on(LedColor color) {
        this._color = color;
    }

    public void off() {
        on(LedColor.Black);
    }

    public void setToAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                on(LedColor.Blue_Chase);
                return;
            }
            else if (DriverStation.getAlliance().get() == Alliance.Red) {
                on(LedColor.Red_Chase);
                return;
            }
        }
        on(LedColor.RAINBOW_PARTY);
    }

    public void setToNote(boolean value){
        if(value){
            this._color = LedColor.Yellow;
        }else{
            setToAlliance();
        }

    }


    @Override
    public void periodic() {
        setToNote(armSubsystem.isHoldingNote());
        _controller.set(this._color.get());
    }
}