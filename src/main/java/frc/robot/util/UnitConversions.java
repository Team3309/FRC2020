package frc.robot.util;

import frc.robot.Config;


public class UnitConversions {
    //Converts encoder ticks to physical distance(inches).
    public static double encoderCountsToInches(double counts) {
        return counts/ Config.DriveWheelEncoderCountsPerRevolution *
                (Math.PI*Config.DriveWheelDiameterInInches);
    }

    //Converts physical distance(inches) to encoder ticks.
    public static double inchesToEncoderCounts(double inches) {
        return inches*(Config.DriveWheelEncoderCountsPerRevolution/
                (Math.PI*Config.DriveWheelDiameterInInches));
    }

    public static double encoderVelocityToInchesPerSecond(double encoderVelocity) {
        return encoderCountsToInches(encoderVelocity * 10.0 / 4096.0 * (Math.PI * Config.DriveWheelDiameterInInches));
    }

    public static double inchesPerSecondToEncoderVelocity(double inchesPerSecond) {
        return inchesToEncoderCounts(inchesPerSecond / 10.0 * 4096.0 / (Math.PI * Config.DriveWheelDiameterInInches));
    }

    public static double degreesPerSecondToEncoderVelocity(double degreesPerSecond) {
        return degreesPerSecond * Config.EncoderCountsPerDegree;
    }

    public static double metersPerSecToInchesPerSec(double metersPerSecond) {
        return metersPerSecond * 39.3701;
    }
}
