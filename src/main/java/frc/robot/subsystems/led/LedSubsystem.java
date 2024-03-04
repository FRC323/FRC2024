package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {
    private final Spark _controller = new Spark(Constants.LED.BLINKIN_PORT);
    private LedColor _color;

    public LedSubsystem() {
        setToAlliance();
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
            }
            else if (DriverStation.getAlliance().get() == Alliance.Red) {
                on(LedColor.Red_Chase);
            }
        }
        on(LedColor.RAINBOW_PARTY);
    }


    @Override
    public void periodic() {
        _controller.set(this._color.get());
    }
}