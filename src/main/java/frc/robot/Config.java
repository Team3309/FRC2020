package frc.robot;

import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.FiringSolutionManager;

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

    //------------------------------------------------------------------------------------------------------------------
    //Hardware Availability//
    //------------------------------------------------------------------------------------------------------------------
    public static Boolean isArmInstalled;
    public static Boolean isClimberInstalled;
    public static Boolean isCtrlPanelInstalled;
    public static Boolean isDriveInstalled;
    public static Boolean isIndexerInstalled;
    public static Boolean isIndexerSensorInstalled;
    public static Boolean isIntakeInstalled;
    public static Boolean isShooterInstalled;
    public static Boolean isVisionInstalled;
    public static Boolean isPcmInstalled;
    public static Boolean isCompressorEnabled;
    public static Boolean isIMUInstalled;
    public static final boolean isDriveAutoTestModeEnabled = false;

    //------------------------------------------------------------------------------------------------------------------
    //Driver Station
    //------------------------------------------------------------------------------------------------------------------
    public static final double operatorControllerDeadzoneRightStick = 0.03;
    public static final double doubleClickSec = 0.4;

    //------------------------------------------------------------------------------------------------------------------
    //Drive Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Drive Motor IDs---------------------------------------------------------------------------------------------------
    public static Integer driveLeftMasterID;
    public static Integer driveLeftSlaveID;
    public static Integer driveRightMasterID;
    public static Integer driveRightSlaveID;

    public static ADIS16470_IMU.IMUAxis imuAxis = ADIS16470_IMU.IMUAxis.kZ;
    public static ADIS16470_IMU.ADIS16470CalibrationTime imuCalibrationTime = ADIS16470_IMU.ADIS16470CalibrationTime._4s;
    public static Double IMUDriftConstant;
    public static Double IMUMountingAngle;

    //Drive Motor PDP Channels------------------------------------------------------------------------------------------
    public static Integer driveLeftMasterPdpChannel;
    public static Integer driveLeftSlavePdpChannel;
    public static Integer driveRightMasterPdpChannel;
    public static Integer driveRightSlavePdpChannel;

    //Drive PID Parameters----------------------------------------------------------------------------------------------
    public static final double driveClosedLoopRampRate = 0.0;
    public static final double driveOpenLoopRampRate = 0.15;
    public static Double driveVelocityP;
    public static Double driveVelocityI;
    public static Integer driveVelocityIntegralZone;
    public static Double driveVelocityD;
    public static Double driveVelocityF;

    //Physical Constants for Drive--------------------------------------------------------------------------------------
    public static Double driveWheelDiameterInInches;
    public static Integer driveWheelEncoderCountsPerRevolution;
    public static Integer driveSpinTurnEncoderCountsPerDegree;
    public static Double driveGearRatio;
    public static final double maxAngularSpeedInDegsPerSec = 360;
    public static final double angCreepSpeedInDegsPerSec = 60;
    public static final double angAccelerationInDegsPerSec2 = 720;
    public static final double angDecelerationInDegsPerSec2 = 720;
    public static final double driveAutoSpinTurnToleranceDegrees = 1.0;
    public static final double maxLinearSpeedInInchesPerSec = 140;  // physical max is about 172
    public static final double linCreepSpeedInInchesPerSec = 30;
    public static final double linAccelerationInInchesPerSec2 = 300;
    public static final double linDecelerationInInchesPerSec2 = 300;
    public static final double linearToleranceInInches = 2.0;
    public static final double linearHeadingCorrectionFactor = 0.1;

    //------------------------------------------------------------------------------------------------------------------
    //Intake Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer intakeMotorID;
    public static Integer intakeMotorPdpChannel;
    public static final double intakeOpenLoopRampRate = 0.5;  // don't strip the belt

    public static Integer intakeSolenoidChannel1;
    public static Integer intakeSolenoidChannel2;

    public static Double intakePistonExtendDelaySeconds;
    public static Double intakePistonRetractDelaySeconds;

    public static final double intakeInwardPower = 0.9;
    public static final double intakeOutwardPower = 0.6;
    public static final double intakeEmergencyOutwardPower = 0.8;

    //------------------------------------------------------------------------------------------------------------------
    //Control Panel Manipulator Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Manipulator Actuator IDs------------------------------------------------------------------------------------------
    public static Integer turnerMotorID;

    //Manipulator Actuator PDP Channels---------------------------------------------------------------------------------
    public static Integer turnerMotorPdpChannel;

    //Manipulator Control Constants-------------------------------------------------------------------------------------
    public static final double turnerRotationPower = 0.4;
    public static final double turnerDriveHoldPower = -0.1;
    public static final int rotationControlSlices = 32;

    //Color Sensor thresholds-------------------------------------------------------------------------------------------
    public static final double colorMin = 400;

    //------------------------------------------------------------------------------------------------------------------
    //Shooter Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Shooter Motor IDs-------------------------------------------------------------------------------------------------
    public static Integer topShooterMotorID;
    public static Integer bottomShooterMotorID;

    //Shooter PDP Channels----------------------------------------------------------------------------------------------
    public static Integer topShooterPdpChannel;
    public static Integer bottomShooterPdpChannel;

    //Shooter Velocity Constants----------------------------------------------------------------------------------------
    public static final double shooterClosedLoopRampRate = 0.65;  // don't strip the belts when spinning up
    public static final double shooterOpenLoopRampRate = 1.0;    // don't strip the belts when stopping flywheels
    public static Double shooterVelocityP;
    public static Double shooterVelocityI;
    public static Integer shooterVelocityIntegralZone;
    public static Double shooterVelocityD;
    public static Double shooterVelocityF;
    public static final double shooterIntakePowerTopMotor = 0.3;
    public static final double shooterIntakePowerBottomMotor = 0.3;

    public static final int shooterSpeedTolerance = 200; //Encoder counts per 100ms

    // Firing solutions
    //FiringSolutionManager.getSingleton().addSolution
    public static final FiringSolution shooterLongRangeSolution = new FiringSolution(
            "Behind Control Panel", 42900, 3000, 16000, 20000);

    public static final FiringSolution shooterMidRangeSolution = new FiringSolution(
            "Starting Line", 47700, 2000, 10000, 20000);

    public static final FiringSolution shooterShortRangeSolution = new FiringSolution(
            "Alliance Wall", 67280, 1000, 4000, 4000);
            //"Alliance Wall", 67280, 4000, 5000, 15000);


    //------------------------------------------------------------------------------------------------------------------
    //Indexer Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Boolean isVelocityModeShooting;
    public static Integer upperIndexerMotorID;
    public static Integer lowerIndexerMotorID;
    public static Integer lowerIndexerMotorPdpChannel;
    public static Integer upperIndexerMotorPdpChannel;
    public static final double indexerOpenLoopRampRate = 1.0;
    public static final double indexerClosedLoopRampRate = 1.0;
    public static Double indexerTopPositionP;
    public static Double indexerBottomPositionP;
    public static Double indexerPositionI;
    public static Integer indexerPositionIntegralZone;
    public static Double indexerPositionD;
    public static Double indexerTopPositionF;
    public static Double indexerBottomPositionF;
    public static Double indexerVelocityP;
    public static Double indexerVelocityI;
    public static Integer indexerVelocityIntegralZone;
    public static Double indexerVelocityD;
    public static Double indexerVelocityF;
    public static Double indexerTopMinimumOutput;
    public static Double indexerBottomMinimumOutput;
    public static int[] indexInEncoderCounts;
    public static int[] indexOutEncoderCounts;
    public static final int indexInSpeed = 2000; // encoder counts per 100ms

    // we don't know what the nominal flywheel speed is when intaking because we use voltage control for intake
    public static final double autoIndexInFlywheelSpeedUpThreshold = 300;  // ignore small speed changes when at full intake speed
    public static final double autoIndexInFlywheelSpeedDropThreshold = 1000;  // require sizable drop to activate indexer

    //Positive power and positive encoder values are for indexing out; negative for indexing in.
    public static final double indexerPeakOutputReverse = -1.0;
    public static final double indexerPeakOutputForward = 1.0;
    public static final double indexerPositionRampSeconds = 0.1;  // time for indexer to switch between stopped and full-speed
    public static Integer indexerPositioningTolerance;
    public static Integer indexerSensorID;
    public static int maxPowerCells = 5;

    //------------------------------------------------------------------------------------------------------------------
    //Arm Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer armMotorId;
    public static Integer armMotorPdpChannel;

    public static Integer maxArmPosition;
    public static Integer minArmPosition;
    public static Integer armPositionIntakeStowedLimit;
    public static Integer armPositionIntakeStowedTarget;
    public static Integer armPositionIntakeStowedUpperLimit;
    public static Integer armControlPanelPosition;
    public static Integer armPositionVision;

    // BEFORE setting armPIDTestMode to true:
    //   Bleed air.
    //   Manually extend the intake.
    //   Put the arm in the lowest physical position against the battery case.
    //   Power cycle the robot.
    //   Inform all operators that ***BEFORE*** every power up of the robot, the following MUST be done:
    //     Manually extend the intake.
    //     Put the arm in the lowest physical position against the battery case.
    //
    // There is no need to change the default intake position because the compressor is disabled while
    // in tuning mode.
    public static final boolean armPIDTuningMode = false;

    public static Double armP;
    public static Double armI;
    public static Integer armIntegralZone;
    public static Double armD;

    public static final double armPeakOutputReverse = -0.3;
    public static final double armPeakOutputForward = 0.5;
    public static final int armAcceleration = 75000; // Ticks per 100ms per sec
    public static final int armCruiseVelocity = 7500; // Ticks per 100ms

    public static final int armPositioningTolerance = 1000; //maximum encoder count difference to be properly in a position
    public static final double armJoystickTiltToPositionFactor = 100;

    //
    //Climber Constants//
    //

    public static Integer climbMotorId;
    public static Integer climbPdpChannel;
    public static Integer climberDeploySolenoidId;
    public static final double climberMaxPower = 0.35;
    public static final double climberDeployTime = 0.7; //Seconds

    public static final double xBoxTriggerButtonThreshold = 0.5;
    public static final int motorControllerConfigTimeoutMs = 25;

    public static Double limelightMountingAngle;
    public static Double limelightMountingHeight;
    public static Double fieldVisionTargetHeight;
    public static Double fieldVisionDepthOfThreePointHoleFromVisionTarget;
    public static Double fieldVisionHeightOfThreePointHoleFromVisionTarget;
    public static Integer driveLeftSlaveID2019_1;
    public static Integer driveLeftSlaveID2019_2;
    public static Integer driveRightSlaveID2019_1;
    public static Integer driveRightSlaveID2019_2;
    public static final double visionDistanceConstant = Math.sqrt(2);
    public static Boolean isLimelightOn;

    public static final String visionFiringSolutionTag = "Vision";
    public static final String longRangeFiringSolutionTag = "Long Range";
    public static final String midRangeFiringSolutionTag = "Mid Range";
    public static final String allianceWallFiringSolutionTag = "Alliance Wall Range";
    public static final String autonomousFiringSolution = "Autonomous";

    private static void frameSpecificConfig() {

        // Values specific to each physical robot

        switch (currentRobot) {
            case Alpha2020:
                isArmInstalled = true;
                isClimberInstalled = true;
                isCtrlPanelInstalled = false;
                isDriveInstalled = true;
                isIndexerInstalled = true;
                isIndexerSensorInstalled = false;
                isIntakeInstalled = true;
                isShooterInstalled = true;
                isVisionInstalled = true;
                isLimelightOn = false;
                isPcmInstalled = true;
                isCompressorEnabled = true;
                isIMUInstalled = true;

                driveLeftMasterID = 4;
                driveLeftSlaveID = 16;
                driveRightMasterID = 15;
                driveRightSlaveID = 1;
                driveLeftMasterPdpChannel = 1;
                driveLeftSlavePdpChannel = 14;
                driveRightMasterPdpChannel = 15;
                driveRightSlavePdpChannel = 0;

                driveWheelDiameterInInches = 6.0;
                driveWheelEncoderCountsPerRevolution = 2048;  // Falcon 500 internal encoder
                driveSpinTurnEncoderCountsPerDegree = 600;

                driveVelocityP = 0.02;
                driveVelocityI = 0.00015;
                driveVelocityIntegralZone = 250;
                driveVelocityD = 0.0006;
                driveVelocityF = 0.002;

                climberDeploySolenoidId = 3;
                climbMotorId = 13;
                climbPdpChannel = 0; // TODO: Get the right channel

                topShooterMotorID = 2;
                bottomShooterMotorID = 10;
                topShooterPdpChannel = 12;
                bottomShooterPdpChannel = 13;

                shooterVelocityP = 0.06;
                shooterVelocityI = 0.0004;
                shooterVelocityIntegralZone = 1000;
                shooterVelocityD = 0.0;
                shooterVelocityF = 0.049;

                intakeMotorID = 20;
                intakeMotorPdpChannel = 11;
                intakeSolenoidChannel1 = 1;
                intakeSolenoidChannel2 = 2;
                intakePistonExtendDelaySeconds = 0.5;
                intakePistonRetractDelaySeconds = 0.5;

                isVelocityModeShooting = true;
                upperIndexerMotorID = 21;
                lowerIndexerMotorID = 22;
                indexerSensorID = 4;
                upperIndexerMotorPdpChannel = 8;
                lowerIndexerMotorPdpChannel = 9;
                indexerTopPositionP = 0.1;
                indexerBottomPositionP = 0.07;
                indexerPositionI = 0.0;
                indexerPositionD = 5.0;
                indexerPositionIntegralZone = 0;
                indexerTopPositionF = 0.26;
                indexerBottomPositionF = 0.18;
                indexerPositioningTolerance = 1000;
                indexerVelocityP = 0.1;
                indexerVelocityI = 0.001;
                indexerVelocityD = 0.0;
                indexerVelocityIntegralZone = 500;
                indexerVelocityF = 0.26;
                indexerTopMinimumOutput = 0.09;
                indexerBottomMinimumOutput = 0.085;


                // We need to fight gravity both ways.
                // There is more slippage at the start of movement as the belts tighten up.
                // Therefore, longer movements have less encoder loss.
                indexInEncoderCounts = new int[] { 8500, 8500, 8500, 8500, 8500, 8500 };
                indexOutEncoderCounts = new int[] { 8500, 8500, 8500, 8500, 8500, 8500 };

                armMotorId = 3;
                armMotorPdpChannel = 3;

                armP = 0.2; // core power (start at .1)
                armI = 2.54972071e-05; // maintain goal position
                armIntegralZone = 5000; // disable I outside of this range
                armD = 10.0; // increase to lower overshoot (start at 0)

                armPositionVision = 50000; //if you update this then you also need to update the limelightMountingAngle
                maxArmPosition = 72690;
                minArmPosition = 1700;
                armPositionIntakeStowedLimit = 25000;
                armPositionIntakeStowedTarget = armPositionIntakeStowedLimit + armPositioningTolerance;
                armPositionIntakeStowedUpperLimit = armPositionIntakeStowedTarget + armPositioningTolerance;

                // Don't exceed max for the control panel position to avoid stalling the motor.
                // Just let the system stabilize between drive pressure and arm repositioning power applied by
                // the arm motor controller in attempting to maintain the last set position.
                armControlPanelPosition = maxArmPosition;

                limelightMountingAngle = -5.0;
                limelightMountingHeight = 33.0; //inches
                fieldVisionTargetHeight = 78.0 + 14.375; //inches
                fieldVisionDepthOfThreePointHoleFromVisionTarget = 29.0;
                fieldVisionHeightOfThreePointHoleFromVisionTarget = 11.0;

                driveGearRatio = 10.7; //might be different for 2020 beta bot

                IMUDriftConstant = 0.0045;
                IMUMountingAngle = 0.0;

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
                isLimelightOn = false;
                isPcmInstalled = false;
                isCompressorEnabled = false;
                isIMUInstalled = true;

                driveLeftMasterID = 6;
                driveLeftSlaveID = 8;
                driveRightMasterID = 5;
                driveRightSlaveID = 7;
                driveLeftMasterPdpChannel = 13;
                driveLeftSlavePdpChannel = 14;
                driveRightMasterPdpChannel = 1;
                driveRightSlavePdpChannel = 2;

                driveWheelDiameterInInches = 3.75;
                driveWheelEncoderCountsPerRevolution = 2048;  // Falcon 500 internal encoder
                driveSpinTurnEncoderCountsPerDegree = 600;

                driveVelocityP = 0.1;
                driveVelocityI = 0.0;
                driveVelocityIntegralZone = 1000;
                driveVelocityD = 0.0;
                driveVelocityF = 0.0;

                IMUDriftConstant = -0.158;

                break;

            case Comp2019:
                isArmInstalled = false;
                isClimberInstalled = false;
                isCtrlPanelInstalled = false;
                isDriveInstalled = true;
                isIndexerInstalled = false;
                isIntakeInstalled = true;
                isShooterInstalled = false;
                isVisionInstalled = true;
                isLimelightOn = true;
                isPcmInstalled = false;
                isCompressorEnabled = false;
                isIMUInstalled = true;

                driveRightMasterID = 1;
                driveRightSlaveID2019_1 = 2;
                driveRightSlaveID2019_2 = 3;
                driveLeftMasterID = 7;
                driveLeftSlaveID2019_1 = 8;
                driveLeftSlaveID2019_2 = 9;

                driveLeftMasterPdpChannel = 0;
                driveLeftSlavePdpChannel = 1;
                driveRightMasterPdpChannel = 15;
                driveRightSlavePdpChannel = 14;

                driveVelocityP = 0.019;
                driveVelocityI = 0.00015;
                driveVelocityIntegralZone = 250;
                driveVelocityD = 0.0006;
                driveVelocityF = 0.002;

                IMUDriftConstant = -0.12;
                IMUMountingAngle = -90.0;

                driveGearRatio = 9.6;
                driveWheelDiameterInInches = 4.0;
                driveWheelEncoderCountsPerRevolution = 4096;
                driveSpinTurnEncoderCountsPerDegree = 600;

                limelightMountingAngle = -5.0;
                limelightMountingHeight = 33.0; //inches
                fieldVisionTargetHeight = 78.0 + 14.375; //inches
                fieldVisionDepthOfThreePointHoleFromVisionTarget = 29.0;
                fieldVisionHeightOfThreePointHoleFromVisionTarget = 11.0;

                armPositionVision = 150000; //if you update this then you also need to update the limelightMountingAngle
                maxArmPosition = 180000;  // physical max = 190000
                minArmPosition = 3000;
                armPositionIntakeStowedLimit = 45000;
                armPositionIntakeStowedTarget = armPositionIntakeStowedLimit + armPositioningTolerance;
                armPositionIntakeStowedUpperLimit = armPositionIntakeStowedTarget + armPositioningTolerance;
                armControlPanelPosition = 74000;

                intakeMotorID = 13;
                turnerMotorPdpChannel = 7;


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

    private static final byte[] Practice2019_MAC =
            {0x00, (byte) 0x80, 0x2F, 0x22, (byte) 0xB0, (byte) 0x6C};


    public enum RobotModel {
        Alpha2020,
        Practice2017,
        Comp2019
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
            } else if (Arrays.equals(rioMAC, Practice2019_MAC)) {
                currentRobot = RobotModel.Comp2019;
            } else {
                StringBuilder foundMAC = new StringBuilder();
                for (byte macOctet: rioMAC) {
                    foundMAC.append(String.format("0x%02X", macOctet));
                    foundMAC.append(" ");
                }
                DriverStation.reportError("Running on unknown roboRIO with MAC " + foundMAC, false);
                System.err.println("Running on unknown roboRIO with MAC " + foundMAC);
                System.exit(-1);  // make the world stop
            }
        } catch (SocketException ex) {
            ex.printStackTrace();
            System.exit(-1);  // make the world stop
        }
        frameSpecificConfig();
    }

}
