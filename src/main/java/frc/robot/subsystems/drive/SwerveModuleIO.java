package frc.robot.subsystems.drive;

public interface SwerveModuleIO {
  public static class SwerveModuleInputs {
    public double turnOutputVolts = 0.0;
    public double turnOutputCurrent = 0.0;
    public double turnEncoderPositionDeg = 0.0;
    public double turnAngleDeg = 0.0;
    public double turnAngularVelocityDegPerSec = 0.0;

    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSecond = 0.0;
    public double driveOutputVolts = 0.0;
    public double driveOutputCurrent = 0.0;
  }

  /*
   Should be called during periodic in order to update all the inputs to the system
  */
  public void updateInputs(SwerveModuleInputs inputs);

  /*
  This writes the outputs to the controllers
  In the case of simulation you can call the controller's calculate methods here
  In the case of real IO you should also write the outputs to the ESCs
   */
  public void setCommandedOutputs(double steeringAngle, double wheelVelocity);

  public void resetSteeringEncoder(double newValue);

  public default void resetSteeringEncoder() {
    resetSteeringEncoder(0);
  }

  public void resetDriveEncoder(double newValue);

  public default void resetDriveEncoder() {
    resetDriveEncoder(0);
  }

  /*
  Useful for any initialization you need
   */
  public boolean initModule();

  /*
  Will be called during periodic
   */
  public void periodic();
}
