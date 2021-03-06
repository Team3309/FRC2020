package frc.robot.util;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is sendIsEnabled.
 */
public class DriveSignal {
    protected double mLeftMotor;
    protected double mRightMotor;
    protected boolean mBrakeMode;

    public DriveSignal(double left, double right) {
        this(left, right, false);
    }

    public DriveSignal(double left, double right, boolean brakeMode) {
        mLeftMotor = left;
        mRightMotor = right;
        mBrakeMode = brakeMode;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0, 0);
    public static DriveSignal BRAKE = new DriveSignal(0, 0, true);

    public double GetLeft() {
        return mLeftMotor;
    }

    public double GetRight() {
        return mRightMotor;
    }

    public boolean GetBrakeMode() {
        return mBrakeMode;
    }

    @Override
    public String toString() {
        return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
    }
}