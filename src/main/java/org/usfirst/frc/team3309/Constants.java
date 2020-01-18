package org.usfirst.frc.team3309;


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

    public static final int DRIVE_LEFT_MASTER_FALCON_ID = 0;
    public static final int DRIVE_LEFT_SLAVE_FALCON_ID = 0;
    public static final int DRIVE_RIGHT_MASTER_FALCON_ID = 0;
    public static final int DRIVE_RIGHT_SLAVE_FALCON_ID = 0;

    /* DRIVEBASE TUNING CONSTANTS */
    public static final int kDriveVelocitySlot = 0;
    public static final double kDriveVelocityP = 0;
    public static final double kDriveVelocityD = 0;
    public static final double kDriveVelocityF = 0;
    public static final int kDrivePositionSlot = 0;
    public static final double kDrivePositionP = 0;
    public static final double kDrivePositionD = 0;
    public static final double DRIVE_CLOSED_LOOP_RAMP_RATE = 0;
    public static final double DRIVE_OPEN_LOOP_RAMP_RATE = 0;
}
