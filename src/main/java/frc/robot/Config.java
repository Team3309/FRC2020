package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;

public class Config {

    /** Robot parameters
     *
     *
     */

    public static double EncoderCountsPerDegree = 600;

    //------------------------------------------------------------------------------------------------------------------
    //Drive Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Drive Motor IDs---------------------------------------------------------------------------------------------------
    public static int DriveLeftMasterID;
    public static int DriveLeftSlaveID;
    public static int DriveRightMasterID;
    public static int DriveRightSlaveID;

    //Drive Motor PDP Channels------------------------------------------------------------------------------------------
    public static int DriveLeftMasterPdpChannel;
    public static int DriveLeftSlavePdpChannel;
    public static int DriveRightMasterPdpChannel;
    public static int DriveRightSlavePdpChannel;

    //Drive PID Parameters----------------------------------------------------------------------------------------------
    public static int DriveClosedLoopRampRate;
    public static int DriveOpenLoopRampRate;
    public static int DriveVelocityP;
    public static int DriveVelocityD;
    public static int DriveVelocityF;

    //Physical Constants for Drive--------------------------------------------------------------------------------------
    public static double DriveWheelDiameterInInches;
    public static double DriveWheelRadiusInInches = DriveWheelDiameterInInches/2;
    public static double DriveWheelInchesPerRevolution = DriveWheelDiameterInInches * Math.PI;
    public static double DriveWheelEncoderCountsPerRevolution = EncoderCountsPerDegree *
            360 * DriveWheelRadiusInInches;
    public static double DrivetrainWidthInInches;

    //------------------------------------------------------------------------------------------------------------------
    //Intake Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static int IntakeMotorID;
    public static int IntakeMotorPdpChannel;
    public static int IntakeSoleoidChannel;
    //------------------------------------------------------------------------------------------------------------------
    //Control Panel Manipulator Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Manipulator Actuator IDs------------------------------------------------------------------------------------------
    public static int TurnerMotorID;
    public static int TurnerTractorPistonID; //Tractor: (retract = draw back, protract = draw forward, ergo tract=draw
    public static int TurnerHeightAdjustmentPistonID;

    //Manipulator Actuator PDP Channels---------------------------------------------------------------------------------
    public static int TurnerMotorPdpChannel;
    public static int TurnerTractorPistonPdpChannel;
    public static int TurnerHeightAdjustmentPistonPdpChannel;

    //Manipulator Physical Constants------------------------------------------------------------------------------------
    public static double TurnerWheelRadiusInches;
    public static double TurnerWheelDiameterInches = TurnerWheelRadiusInches*2;
    public static double TurnerWheelInchesPerRevolution = TurnerWheelDiameterInches * Math.PI;

    //Manipulator Control Constants-------------------------------------------------------------------------------------
    public static double TurnerRotationSpeed = 0.8;

    //------------------------------------------------------------------------------------------------------------------
    //Shooter Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Shooter Motor IDs-------------------------------------------------------------------------------------------------
    public static int TopShooterMotorID;
    public static int BottomShooterMotorID;

    //Shooter PDP Channels----------------------------------------------------------------------------------------------
    public static int TopShooterPdpChannel;
    public static int BottomShooterPdpChannel;

    //Shooter PID Constants---------------------------------------------------------------------------------------------
    public static double ShooterClosedLoopRampRate;
    public static double ShooterOpenLoopRampRate;
    public static double ShooterVelocityP;
    public static double ShooterVelocityI;
    public static int ShooterVelocityIntegralZone;
    public static double ShooterVelocityD;
    public static double ShooterVelocityF;

    /* Default Shooter timeout */
    public static double DefaultShooterTimeout;

    //------------------------------------------------------------------------------------------------------------------
    //Indexer Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static int IndexerMotorID;
    public static int IndexerPdpChannel;

    //------------------------------------------------------------------------------------------------------------------
    //Balancer Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static int BalancerMotorId;
    public static int BalancerPdpChannel;

    //------------------------------------------------------------------------------------------------------------------
    //Aiming Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static double AimingP;
    public static double AimingI;
    public static double AimingD;
    public static double AimingF;

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
