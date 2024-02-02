package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import java.util.function.BooleanSupplier;

//Largely borrowed from 3005's code
public class SparkMaxUtils {
    public static int check(REVLibError error) { return error == REVLibError.kOk ? 0 : 1; }

    public static class UnitConversions {

        public static REVLibError setDegreesFromGearRatio(
                AbsoluteEncoder sparkMaxEncoder, double ratio) {
            double degreesPerRotation = 360.0 / ratio;
            double degreesPerRotationPerSecond = degreesPerRotation / 60.0;
            REVLibError error = sparkMaxEncoder.setPositionConversionFactor(degreesPerRotation);

            if (error != REVLibError.kOk) {
                return error;
            }

            return sparkMaxEncoder.setVelocityConversionFactor(degreesPerRotationPerSecond);
        }

        public static REVLibError setRadsFromGearRatio(AbsoluteEncoder sparkMaxEncoder, double ratio) {
            double radsPerRotation = (2.0 * Math.PI) / ratio;
            double radsPerRotationPerSecond = radsPerRotation / 60.0;
            REVLibError error = sparkMaxEncoder.setPositionConversionFactor(radsPerRotation);

            if (error != REVLibError.kOk) {
                return error;
            }

            return sparkMaxEncoder.setVelocityConversionFactor(radsPerRotationPerSecond);
        }

        public static REVLibError setDegreesFromGearRatio(
                RelativeEncoder sparkMaxEncoder, double ratio) {
            double degreesPerRotation = 360.0 / ratio;
            double degreesPerRotationPerSecond = degreesPerRotation / 60.0;
            REVLibError error = sparkMaxEncoder.setPositionConversionFactor(degreesPerRotation);

            if (error != REVLibError.kOk) {
                return error;
            }

            return sparkMaxEncoder.setVelocityConversionFactor(degreesPerRotationPerSecond);
        }

        public static REVLibError setRadsFromGearRatio(RelativeEncoder sparkMaxEncoder, double ratio) {
            double radsPerRotation = (2.0 * Math.PI) / ratio;
            double radsPerRotationPerSecond = radsPerRotation / 60.0;
            REVLibError error = sparkMaxEncoder.setPositionConversionFactor(radsPerRotation);

            if (error != REVLibError.kOk) {
                return error;
            }

            return sparkMaxEncoder.setVelocityConversionFactor(radsPerRotationPerSecond);
        }
    }
    public static boolean initWithRetry(BooleanSupplier initFunction, int maxRetryAttempts) {
        int numAttempts = 0;
        while (!initFunction.getAsBoolean()) {
            numAttempts++;
            if (numAttempts > maxRetryAttempts) {
                return false;
            }
        }
    return true;
    }
}
