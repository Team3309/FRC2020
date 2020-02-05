package frc.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;

/*
 * This class holds all the key values that control how the robot functions.
 * The values are kept in one place so its easy to find them when you need to make a change.
 */
public class Constants {

    public static final int TURNER_MOTOR_ID = 20;
    public static final int TURNER_RETRACTOR_PISTON_ID = 0;
    public static final int TURNER_HEIGHT_ADJUST_PISTON_ID = 1;
    public static final int INDEXER_MOTOR_ID = 21;
    public static final int INTAKE_MOTOR_ID = 22;
    public static final int BALANCER_MOTOR_ID = 23;
    public static final int SHOOTER_TOP_MOTOR_ID = 2;
    public static final int SHOOTER_BOTTOM_MOTOR_ID = 3;

    /**
     * Motor and physical constants.
     *
     *
     */
    public static final int DRIVE_CLOSED_LOOP_RAMP_RATE = 0;
    public static final int DRIVE_OPEN_LOOP_RAMP_RATE = 0;
    public static final int INTAKE_MOTOR_STANDARD_VELOCITY = 0;

    public static final int kDriveVelocityP = 0;
    public static final int kDriveVelocityD = 0;
    public static final int kDriveVelocityF = 0;
    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 0;
    public static final double DRIVE_WHEEL_RADIUS_INCHES = DRIVE_WHEEL_DIAMETER_INCHES/2;
    public static final double DRIVE_INCHES_PER_REV = (DRIVE_WHEEL_DIAMETER_INCHES*Math.PI);
    public static final double DRIVE_ENCODER_COUNTS_PER_REV = 0;

    public static final double TURNER_WHEEL_DIAMETER_INCHES = 0;
    public static final double TURNER_INCHES_PER_REV = TURNER_WHEEL_DIAMETER_INCHES*Math.PI;

    public static final double  DRIVETRAIN_WIDTH = 0;

    public static final double ENCODER_COUNTS_PER_DEGREE = 0;

    //All pdp channel numbers are placeholders for now.
    public static final int kTurnerRetractorPistonPdpChannel = 4;
    public static final int kTurnerHeightAdjustmentPistonPdpChannel = 5;
    public static final int kTurnerMotorPdpChannel = 6;
    public static final int kIntakeMotorPdpChannel = 7;
    public static final int kIndexerMotorPdpChannel = 8;
    public static final int kTopShooterMotorPdpChannel = 15;
    public static final int kBottomShooterMotorPdpChannel = 0;
    public static final int kBalancerMotorPdpChannel = 11;

    /* SHOOTER CONFIG CONSTANTS */

    public static final double kShooterClosedLoopRampRate = 0;
    public static final double kShooterOpenLoopRampRate = 0;
    public static final double kShooterVelocityP = 0;
    public static final double kShooterVelocityI = 0;
    public static final int kShooterVelocityIntegralZone = 0;
    public static final double kShooterVelocityD = 0;
    public static final double kShooterVelocityF = 0;
    public static final double kShooterGearRatio = 16.0/24.0;

    /* AIMING CONSTANTS */
    public static final double kAimingP = 0;
    public static final double kAimingI = 0;
    public static final double kAimingD = 0;

    /* CONTROL PANEL CONSTANTS */
    public static final double kTurnerRotationSpeed = 0.8;
}
