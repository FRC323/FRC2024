package frc.robot.subsystems.drive;


import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements Sendable {

  private SwerveModuleIO swerveModuleIO;
  //    Radians offset for the module
  private Rotation2d moduleOffset;
  private final boolean isInverted;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d(0));

  public SwerveModule(
      int drivingCanId,
      int turningCanId,
      double moduleOffset,
      boolean isInverted,
      boolean isSimulation) {
    if (isSimulation) {
      swerveModuleIO = new SwerveModuleIOSim(isInverted);
    } else {
      swerveModuleIO = new SwerveModuleIOSparkMax(drivingCanId, turningCanId, moduleOffset, isInverted);
    }
    this.moduleOffset = new Rotation2d(moduleOffset);
    this.isInverted = isInverted;

    swerveModuleIO.initModule();
    //    TODO: This maybe should be moved out of the IO and into the SparkMax implementation
    swerveModuleIO.burnFlashSparks();

    desiredState.angle = swerveModuleIO.getModuleHeading().minus(this.moduleOffset);
    swerveModuleIO.resetDriveEncoder(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        swerveModuleIO.getVelocity(), swerveModuleIO.getModuleHeading().minus(moduleOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        swerveModuleIO.getPosition(), swerveModuleIO.getModuleHeading().minus(moduleOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(moduleOffset);
    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(correctedDesiredState, swerveModuleIO.getModuleHeading());
    this.desiredState = optimizedState;
    swerveModuleIO.setCommandedOutputs(optimizedState.angle, optimizedState.speedMetersPerSecond);
  }

  public double getEncoderAbsPositionRad() {
    return swerveModuleIO.getModuleHeading().getRadians();
  }

  public void periodic() {
    swerveModuleIO.periodic();
  }

  public void setModuleOffset(double offset) {
    this.moduleOffset = new Rotation2d(offset);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    swerveModuleIO.initSendable(builder);
  }
}
