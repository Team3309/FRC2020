package frc.robot.util;

import frc.robot.Config;


public class UnitConversions {
    //Converts encoder ticks to physical distance(inches).
    public static double EncoderCountsToInches(double counts) {
        return counts/ Config.DriveWheelEncoderCountsPerRevolution *
                (Math.PI*Config.DriveWheelDiameterInInches);
    }

    //Converts physical distance(inches) to encoder ticks.
    public static double InchesToEncoderCounts(double inches) {
        return inches*(Config.DriveWheelEncoderCountsPerRevolution/
                (Math.PI*Config.DriveWheelDiameterInInches));
    }

    public static double EncoderVelocityToInchesPerSecond(double encoderVelocity) {
        return EncoderCountsToInches(encoderVelocity * 10.0 / 4096.0 * (Math.PI * Config.DriveWheelDiameterInInches));
    }

    public static double InchesPerSecondToEncoderVelocity(double inchesPerSecond) {
        return InchesToEncoderCounts(inchesPerSecond / 10.0 * 4096.0 / (Math.PI * Config.DriveWheelDiameterInInches));
    }

    public static double DegreesPerSecondToEncoderVelocity(double degreesPerSecond) {
        return degreesPerSecond * Config.EncoderCountsPerDegree;
    }

    public static double MetersPerSecToInchesPerSec(double metersPerSecond) {
        return metersPerSecond * 39.3701;
    }
}
