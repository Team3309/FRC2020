package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;

public class Config {

    /** Robot parameters
     *
     * Use final primitives for constant values that are the same for all frames.
     *
     * Use class wrappers (Integer, Double) for configurable values so that a null pointer exception
     * is thrown if a value is not initialized by the frame specific code.
     *
     */

    public static final boolean isInDebug = false;
    public static final boolean isTestMode = true;  // activate use of RobotContainerTest

    //------------------------------------------------------------------------------------------------------------------
    //Hardware Availability//
    //------------------------------------------------------------------------------------------------------------------
    public static Boolean isArmInstalled;
    public static Boolean isClimberInstalled;
    public static Boolean isCtrlPanelInstalled;
    public static Boolean isDriveInstalled;
    public static Boolean isIndexerInstalled;
    public static Boolean isIntakeInstalled;
    public static Boolean isShooterInstalled;
    public static Boolean isVisionInstalled;

    //------------------------------------------------------------------------------------------------------------------
    //Drive Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Drive Motor IDs---------------------------------------------------------------------------------------------------
    public static Integer DriveLeftMasterID;
    public static Integer DriveLeftSlaveID;
    public static Integer DriveRightMasterID;
    public static Integer DriveRightSlaveID;

    //Drive Motor PDP Channels------------------------------------------------------------------------------------------
    public static Integer DriveLeftMasterPdpChannel;
    public static Integer DriveLeftSlavePdpChannel;
    public static Integer DriveRightMasterPdpChannel;
    public static Integer DriveRightSlavePdpChannel;

    //Drive PID Parameters----------------------------------------------------------------------------------------------
    public static final double driveClosedLoopRampRate = 0.0;
    public static final double driveOpenLoopRampRate = 0.15;
    public static Double driveVelocityP;
    public static Double driveVelocityI;
    public static Integer driveVelocityIntegralZone;
    public static Double driveVelocityD;
    public static Double driveVelocityF;

    //Physical Constants for Drive--------------------------------------------------------------------------------------
    public static final double DriveWheelDiameterInInches = 6.0;
    public static final double EncoderCountsPerDegree = 600;
    public static final double DriveWheelRadiusInInches = DriveWheelDiameterInInches/2;
    public static final double DriveWheelInchesPerRevolution = DriveWheelDiameterInInches * Math.PI;
    public static final double DriveWheelEncoderCountsPerRevolution = EncoderCountsPerDegree *
            360 * DriveWheelRadiusInInches;
    //public static final double DrivetrainWidthInInches = ?;

    //------------------------------------------------------------------------------------------------------------------
    //Intake Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer IntakeMotorID;
    public static Integer IntakeMotorPdpChannel;
    public static Integer IntakeSoleoidChannel;
    public static final double intakeInwardPower = 0.3;
    public static final double intakeOutwardPower = 0.3;

    //------------------------------------------------------------------------------------------------------------------
    //Control Panel Manipulator Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Manipulator Actuator IDs------------------------------------------------------------------------------------------
    public static Integer TurnerMotorID;
    public static Integer TurnerTractorPistonID; //Tractor: (retract = draw back, protract = draw forward, ergo tract=draw
    public static Integer TurnerHeightAdjustmentPistonID;

    //Manipulator Actuator PDP Channels---------------------------------------------------------------------------------
    public static Integer TurnerMotorPdpChannel;

    //Manipulator Physical Constants------------------------------------------------------------------------------------
    public static final double TurnerWheelRadiusInches = 4.0;  //  value?
    public static final double TurnerWheelDiameterInches = TurnerWheelRadiusInches*2;
    public static final double TurnerWheelInchesPerRevolution = TurnerWheelDiameterInches * Math.PI;

    //Manipulator Control Constants-------------------------------------------------------------------------------------
    public static final double TurnerRotationPower = 0.4;

    //------------------------------------------------------------------------------------------------------------------
    //Shooter Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Shooter Motor IDs-------------------------------------------------------------------------------------------------
    public static Integer TopShooterMotorID;
    public static Integer BottomShooterMotorID;

    //Shooter PDP Channels----------------------------------------------------------------------------------------------
    public static Integer TopShooterPdpChannel;
    public static Integer BottomShooterPdpChannel;

    //Shooter Velocity Constants----------------------------------------------------------------------------------------
    public static final double shooterClosedLoopRampRate = 0.0;
    public static final double shooterOpenLoopRampRate = 0.15;
    public static Double shooterVelocityP;
    public static Double shooterVelocityI;
    public static Integer shooterVelocityIntegralZone;
    public static Double shooterVelocityD;
    public static Double shooterVelocityF;
    public static Double shooterStandardVelocity;
    public static Double shooterStandardTimeout;

    //------------------------------------------------------------------------------------------------------------------
    //Indexer Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer IndexerMotorID;
    public static Integer IndexerPdpChannel;
    public static Double IndexerStandardVelocity;

    //------------------------------------------------------------------------------------------------------------------
    //Balancer Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer BalancerMotorId;
    public static Integer BalancerPdpChannel;

    //------------------------------------------------------------------------------------------------------------------
    //Arm Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer ArmMotorId;
    public static Integer ArmMotorPdpChannel;
    public static Integer ArmMaxAnglePosition;
    public static Integer ArmLongRangeAnglePosition;
    public static Integer ArmMidRangeAnglePosition;
    public static Integer ArmCloseRangeAnglePosition;
    public static Integer ArmTrenchDriveAnglePosition;
    public static Integer ArmMinAnglePosition;

    //------------------------------------------------------------------------------------------------------------------
    //Aiming Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Double AimingP;
    public static Double AimingI;
    public static Double AimingD;
    public static Double AimingF;


    private static void frameSpecificConfig() {

        // Values specific to each physical robot

        switch (currentRobot) {
            case Alpha2020:
                isArmInstalled = false;
                isClimberInstalled = false;
                isCtrlPanelInstalled = false;
                isDriveInstalled = true;
                isIndexerInstalled = false;
                isIntakeInstalled = true;
                isShooterInstalled = false;
                isVisionInstalled = false;

                DriveLeftMasterID = 4;
                DriveLeftSlaveID = 16;
                DriveRightMasterID = 15;
                DriveRightSlaveID = 1;
                DriveLeftMasterPdpChannel = 14;
                DriveLeftSlavePdpChannel = 15;
                DriveRightMasterPdpChannel = 0;
                DriveRightSlavePdpChannel = 1;

                driveVelocityP = 0.1;
                driveVelocityI = 0.0;
                driveVelocityIntegralZone = 1000;
                driveVelocityD = 0.0;
                driveVelocityF = 0.0;

                shooterVelocityP = 0.1;
                shooterVelocityI = 0.0;
                shooterVelocityIntegralZone = 1000;
                shooterVelocityD = 0.0;
                shooterVelocityF = 0.0;
                break;

            case Practice2017:
                isArmInstalled = false;
                isClimberInstalled = false;
                isCtrlPanelInstalled = false;
                isDriveInstalled = true;
                isIndexerInstalled = false;
                isIntakeInstalled = false;
                isShooterInstalled = false;
                isVisionInstalled = false;

                DriveLeftMasterID = 6;
                DriveLeftSlaveID = 8;
                DriveRightMasterID = 5;
                DriveRightSlaveID = 7;
                DriveLeftMasterPdpChannel = 13;
                DriveLeftSlavePdpChannel = 14;
                DriveRightMasterPdpChannel = 1;
                DriveRightSlavePdpChannel = 2;

                driveVelocityP = 0.1;
                driveVelocityI = 0.0;
                driveVelocityIntegralZone = 1000;
                driveVelocityD = 0.0;
                driveVelocityF = 0.0;

                shooterVelocityP = 0.1;
                shooterVelocityI = 0.0;
                shooterVelocityIntegralZone = 1000;
                shooterVelocityD = 0.0;
                shooterVelocityF = 0.0;
                break;
        }
    }

    /*
     * These MAC_ADDR values are just unique IDs for each of the roboRIOs
     * They are used to identify which robot the code is running on, because some values are specific to each robot.
     */
    private static final byte[] Alpha2020_MAC = {
            (byte) 0x00, (byte) 0x80, (byte) 0x2F, (byte) 0x19, (byte) 0x57, (byte) 0x03};

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
                System.err.println("Running on unknown roboRIO with MAC " + foundMAC);
                int a = 1 / 0;  // make the world stop
            }
        } catch (SocketException ex) {
            ex.printStackTrace();
            int a = 1 / 0;  // make the world stop
        }
        frameSpecificConfig();
    }

}
