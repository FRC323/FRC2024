package frc.robot.subsystems.vision;

public class LimelightOptions {
    public enum VisionMode {
        VisionProcessing(0),
        DriverCamera(1);

        private final int mode;

        VisionMode(int mode) {
            this.mode = mode;
        }

        public int get() {
            return mode;
        }
    }
}
