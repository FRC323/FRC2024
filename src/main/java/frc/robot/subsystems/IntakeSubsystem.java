package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.utils.SparkMaxUtils.check;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax feederRoller;

    public IntakeSubsystem(){
        feederRoller = new CANSparkMax(Constants.Intake.ROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
        check(feederRoller.restoreFactoryDefaults());
    }

    public void setIntakeSpeed(double vel) {
        this.feederRoller.set(vel);
    }
}
