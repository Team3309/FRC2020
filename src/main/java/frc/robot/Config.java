package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;

public class Config {

    // Robot parameters

    public static int DriveLeftMasterID;
    public static int DriveLeftSlaveID;
    public static int DriveRightMasterID;
    public static int DriveRightSlaveID;

    public static int DriveLeftMasterPdpChannel;
    public static int DriveLeftSlavePdpChannel;
    public static int DriveRightMasterPdpChannel;
    public static int DriveRightSlavePdpChannel;

    /*
     * These MAC_ADDR values are just unique IDs for each of the roboRIOs
     * They are used to identify which robot the code is running on, because some values are specific to each robot.
     */
    private static final byte[] Alpha2020_MAC = {
            (byte) 0x00, (byte) 0x80, (byte) 0x2F, (byte) 0x25, (byte) 0x13, (byte) 0xAA}; // needs update!

    private static final byte[] Practice2017_MAC = {
            (byte) 0x00, (byte) 0x80, (byte) 0x2F, (byte) 0x25, (byte) 0x13, (byte) 0x96};

    public enum RobotModel {
        Alpha2020,
        Practice2017
    }

    /*
     * This holds an instance of the type we defined above.
     * The static block beneath it just handles fetching the address of the rio we are running on
     * and sets currentRobot to the appropriate value
     */
    public static RobotModel currentRobot;

    static {
        try {
            byte[] rioMAC = NetworkInterface.getByName("eth0").getHardwareAddress();
            if (Arrays.equals(rioMAC, Alpha2020_MAC)) {
                currentRobot = RobotModel.Alpha2020;
            } else if (Arrays.equals(rioMAC, Practice2017_MAC)) {
                currentRobot = RobotModel.Practice2017;
            } else {
                String foundMAC = "";
                for (int i = 0; i < rioMAC.length; i++) {
                    foundMAC = foundMAC + String.format("0x%02X", rioMAC[i]) + " ";
                }
                DriverStation.reportError("Running on unknown roboRIO with MAC " + foundMAC, false);
                int a = 1 / 0;  // make the world stop
            }
        } catch (SocketException ex) {
            ex.printStackTrace();
            int a = 1 / 0;  // make the world stop
        }

        // Values specific to each physical robot

        switch (currentRobot) {
            case Alpha2020:
                DriveLeftMasterID = 4;
                DriveLeftSlaveID = 16;
                DriveRightMasterID = 15;
                DriveRightSlaveID = 1;
                DriveLeftMasterPdpChannel = 14;
                DriveLeftSlavePdpChannel = 15;
                DriveRightMasterPdpChannel = 0;
                DriveRightSlavePdpChannel = 1;
                break;

            case Practice2017:
                DriveLeftMasterID = 6;
                DriveLeftSlaveID = 8;
                DriveRightMasterID = 5;
                DriveRightSlaveID = 7;
                DriveLeftMasterPdpChannel = 13;
                DriveLeftSlavePdpChannel = 14;
                DriveRightMasterPdpChannel = 1;
                DriveRightSlavePdpChannel = 2;
                break;
        }
    }

}
