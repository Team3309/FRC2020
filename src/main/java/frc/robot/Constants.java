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

    public static final int DriveVelocityP = 0;
    public static final int DriveVelocityD = 0;
    public static final int DriveVelocityF = 0;
    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 0;
    public static final double DRIVE_WHEEL_RADIUS_INCHES = DRIVE_WHEEL_DIAMETER_INCHES/2;
    public static final double DRIVE_INCHES_PER_REV = (DRIVE_WHEEL_DIAMETER_INCHES*Math.PI);
    public static final double DRIVE_ENCODER_COUNTS_PER_REV = 0;

    public static final double TURNER_WHEEL_DIAMETER_INCHES = 0;
    public static final double TURNER_INCHES_PER_REV = TURNER_WHEEL_DIAMETER_INCHES*Math.PI;

    public static final double  DRIVETRAIN_WIDTH = 0;

    public static final double ENCODER_COUNTS_PER_DEGREE = 0;

    //All pdp channel numbers are placeholders for now.
    public static final int LeftDriveMasterPdpChannel = 1;
    public static final int LeftDriveSlavePdpChannel = 2;
    public static final int RightDriveMasterPdpChannel = 13;
    public static final int RightDriveSlavePdpChannel = 14;
    public static final int TurnerRetractorPistonPdpChannel = 4;
    public static final int TurnerHeightAdjustmentPistonPdpChannel = 5;
    public static final int TurnerMotorPdpChannel = 6;
    public static final int IntakeMotorPdpChannel = 7;
    public static final int IndexerMotorPdpChannel = 8;
    public static final int TopShooterMotorPdpChannel = 15;
    public static final int BottomShooterMotorPdpChannel = 0;
    public static final int BalancerMotorPdpChannel = 11;
    public static final int DriveVelocitySlot = 0;
    public static final int DrivePositionSlot = 0;
    public static final double DrivePositionP = 0;
    public static final double DrivePositionD = 0;

    /* SHOOTER CONFIG CONSTANTS */

    public static final double ShooterClosedLoopRampRate = 0;
    public static final double ShooterOpenLoopRampRate = 0;
    public static final double ShooterVelocityP = 0;
    public static final double ShooterVelocityI = 0;
    public static final int ShooterVelocityIntegralZone = 0;
    public static final double ShooterVelocityD = 0;
    public static final double ShooterVelocityF = 0;

    /* AIMING CONSTANTS */
    public static final double AimingP = 0;
    public static final double AimingI = 0;
    public static final double AimingD = 0;

}
