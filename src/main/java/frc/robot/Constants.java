package frc.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;

/*
 * This class holds all the key values that control how the robot functions.
 * The values are kept in one place so its easy to find them when you need to make a change.
 */
public class Constants {

    /*
     * These MAC_ADDR values are just unique IDs for each of the roboRIO's used in 2020
     * They are used to identify which robot the code is running on, because some values are specific to each robot.
     */
    private static final byte[] PRACTICEBOT_MAC_ADDR = {0x00, (byte) 0x80, 0x2F, 0x17, (byte) 0x85, (byte) 0xD3};
    private static final byte[] COMPBOT_MAC_ADDR = {0x00, (byte) 0x80, 0x2F, 0x22, (byte) 0xB0, (byte) 0x6C}; // find this at comp

    /*
     * This enum defines a type with 2 values, PRACTICE and COMPETITION
     * These values represent our COMPETITION robot and PRACTICE robot.
     */
    public enum Robot {
        PRACTICE,
        COMPETITION
    }

    /*
     * This holds an instance of the type we defined above.
     * The static block beneath it just handles fetching the address of the rio we are running on
     * and sets currentRobot to the appropriate value
     */
    public static Robot currentRobot;

    static {
        try {
            byte[] rioMac = NetworkInterface.getByName("eth0").getHardwareAddress();
            if (Arrays.equals(rioMac, PRACTICEBOT_MAC_ADDR)) {
                currentRobot = Robot.PRACTICE;
            } else if (Arrays.equals(rioMac, COMPBOT_MAC_ADDR)) {
                currentRobot = Robot.COMPETITION;
            } else {
                currentRobot = null;
                System.err.println("Oh no! Unknown robot! Did somebody install a new rio?");
            }
        } catch (SocketException ex) {
            ex.printStackTrace();
        }
    }

    /* DRIVETRAIN */
    public static final int DRIVE_RIGHT_MASTER_ID = 5;
    public static final int DRIVE_RIGHT_SLAVE_ID = 7;
    public static final int DRIVE_LEFT_MASTER_ID = 6;
    public static final int DRIVE_LEFT_SLAVE_ID = 8;
    public static final int TURNER_MOTOR_ID = 20;
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
    public static final int kLeftDriveMasterPdpChannel = 1;
    public static final int kLeftDriveSlavePdpChannel = 2;
    public static final int kRightDriveMasterPdpChannel = 13;
    public static final int kRightDriveSlavePdpChannel = 14;
    public static final int kTurnerRetractorPistonPdpChannel = 4;
    public static final int kTurnerHeightAdjustmentPistonPdpChannel = 5;
    public static final int kTurnerMotorPdpChannel = 6;
    public static final int kIntakeMotorPdpChannel = 7;
    public static final int kIndexerMotorPdpChannel = 8;
    public static final int kTopShooterMotorPdpChannel = 15;
    public static final int kBottomShooterMotorPdpChannel = 0;
    public static final int kBalancerMotorPdpChannel = 11;
    public static final int kDriveVelocitySlot = 0;
    public static final int kDrivePositionSlot = 0;
    public static final double kDrivePositionP = 0;
    public static final double kDrivePositionD = 0;

    /* SHOOTER CONFIG CONSTANTS */

    public static final double kShooterClosedLoopRampRate = 0;
    public static final double kShooterOpenLoopRampRate = 0;
    public static final double kShooterVelocityP = 0;
    public static final double kShooterVelocityI = 0;
    public static final int kShooterVelocityIntegralZone = 0;
    public static final double kShooterVelocityD = 0;
    public static final double kShooterVelocityF = 0;

    /* AIMING CONSTANTS */
    public static final double kAimingP = 0;
    public static final double kAimingI = 0;
    public static final double kAimingD = 0;

}
