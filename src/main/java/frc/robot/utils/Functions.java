package frc.robot.utils;

public class Functions {
    public static boolean contains(int[] array, int target) {
        for (int num : array) {
            if (num == target) {
                return true;
            }
        }
        return false;
    }

    public static boolean isInRange(double value, double minValue, double maxValue) {
        if (minValue > value) {
            return false;
        } else if (maxValue < value) {
            return false;
        } else {
            return true;
        }
    }
}
