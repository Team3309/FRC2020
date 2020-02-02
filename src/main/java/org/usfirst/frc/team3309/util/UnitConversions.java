package org.usfirst.frc.team3309.util;

import org.usfirst.frc.team3309.Constants;

public class UnitConversions {
    //Converts encoder ticks to physical distance(inches).
    public static double EncoderCountsToInches(double counts) {
        return counts/ Constants.DRIVE_ENCODER_COUNTS_PER_REV * (Math.PI*Constants.DRIVE_WHEEL_DIAMETER_INCHES);
    }

    //Converts physical distance(inches) to encoder ticks.
    public static double InchesToEncoderCounts(double inches) {
        return inches*(Constants.DRIVE_ENCODER_COUNTS_PER_REV/(Math.PI*Constants.DRIVE_WHEEL_DIAMETER_INCHES));
    }

    public static double EncoderVelocityToInchesPerSecond(double encoderVelocity) {
        return EncoderCountsToInches(encoderVelocity * 10.0 / 4096.0 * (Math.PI * Constants.DRIVE_WHEEL_DIAMETER_INCHES));
    }

    public static double InchesPerSecondToEncoderVelocity(double inchesPerSecond) {
        return InchesToEncoderCounts(inchesPerSecond / 10.0 * 4096.0 / (Math.PI * Constants.DRIVE_WHEEL_DIAMETER_INCHES));
    }

    public static double DegreesPerSecondToEncoderVelocity(double degreesPerSecond) {
        return degreesPerSecond * Constants.ENCODER_COUNTS_PER_DEGREE;
    }

    public static double MetersPerSecToInchesPerSec(double metersPerSecond) {
        return metersPerSecond * 39.3701;
    }
}
