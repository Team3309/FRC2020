package frc.robot;

import com.analog.adis16470.frc.ADIS16470_IMU;
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

    //------------------------------------------------------------------------------------------------------------------
    //Intake Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer intakeMotorID;
    public static Integer intakeMotorPdpChannel;
    public static final double intakeOpenLoopRampRate = 1.0;  // don't strip the belt

    public static Integer intakeSolenoidChannel1;
    public static Integer intakeSolenoidChannel2;

    public static Double intakePistonExtendDelaySeconds;
    public static Double intakePistonRetractDelaySeconds;

    public static final double intakeInwardPower = 0.6;
    public static final double intakeOutwardPower = 0.3;

    //------------------------------------------------------------------------------------------------------------------
    //Control Panel Manipulator Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Manipulator Actuator IDs------------------------------------------------------------------------------------------
    public static Integer turnerMotorID;

    //Manipulator Actuator PDP Channels---------------------------------------------------------------------------------
    public static Integer turnerMotorPdpChannel;

    //Manipulator Control Constants-------------------------------------------------------------------------------------
    public static final double turnerRotationPower = 0.4;
    public static final double turnerHoldPower = -0.1;
    public static final int rotationControlSlices = 32;

    //Color Sensor thresholds-------------------------------------------------------------------------------------------
    public static final int colorThreshold = 235; //Color value must be above this to be recognized
    public static final int colorEpsilon = 50; //Threshold for deciding which color is in front of the sensor

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
    public static final double shooterClosedLoopRampRate = 1.0;  // don't strip the belts when spinning up
    public static final double shooterOpenLoopRampRate = 1.0;    // don't strip the belts when stopping flywheels
    public static Double shooterVelocityP;
    public static Double shooterVelocityI;
    public static Integer shooterVelocityIntegralZone;
    public static Double shooterVelocityD;
    public static Double shooterVelocityF;
    public static final double shooterIntakePowerTopMotor = 0.4;
    public static final double shooterIntakePowerBottomMotor = 0.4;

    public static final int shooterSpeedTolerance = 100; //Encoder counts per 100ms

    public static final FiringSolution shooterLongRangeSolution = new FiringSolution(
            "Long Range", 90000, 5000, 21300, 21300);
    public static final FiringSolution shooterMidRangeSolution = new FiringSolution(
            "Mid Range", 103000, 5000, 18000, 21300);
    public static final FiringSolution shooterShortRangeSolution = new FiringSolution(
            "Short Range", 159447, 5000, 5000, 20000);

    //------------------------------------------------------------------------------------------------------------------
    //Indexer Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer upperIndexerMotorID;
    public static Integer lowerIndexerMotorID;
    public static Integer lowerIndexerMotorPdpChannel;
    public static Integer upperIndexerMotorPdpChannel;
    public static final double indexerOpenLoopRampRate = 1.0;
    public static final double indexerClosedLoopRampRate = 1.0;
    public static Double indexerP;
    public static Double indexerI;
    public static Integer indexerIntegralZone;
    public static Double indexerD;
    public static Double indexerF;

    //Positive power and positive encoder values are for indexing out; negative for indexing in.
    public static final double indexerPeakOutputReverse = -1.0;
    public static final double indexerPeakOutputForward = 1.0;
    public static final int indexerAcceleration = 20000;
    public static final int indexerCruiseVelocity = 1000;
    public static Integer powerCellDistanceInEncoderCounts;
    public static Integer indexerPositioningTolerance;
    public static Integer indexerSensorID;
    public static int maxPowerCells = 5;

    //------------------------------------------------------------------------------------------------------------------
    //Arm Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer armMotorId;
    public static Integer armHallEffectLimitSwitchId;
    public static Integer armMotorPdpChannel;

    public static Integer maxArmPosition;
    public static Integer trenchArmPosition;
    public static Integer minArmPosition;
    public static Integer armPositionHallEffectTop;
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

    // The procedure for running the arm without position sensors is similar to armPIDTestMode
    // and just as dangerous, but we suppress warnings in this mode because we're willing to
    // play with fire to make progress.
    //
    // BEFORE setting armNoPositionSensors to true:
    //   Inform all operators that ***BEFORE*** every power up of the robot, the following MUST be done:
    //     Manually extend the intake.
    //     Put the arm in the lowest physical position against the battery case.
    public static final boolean armNoPositionSensors = true;

    public static Double armP;
    public static Double armI;
    public static Integer armIntegralZone;
    public static Double armD;

    public static final double armPeakOutputReverse = -0.2;
    public static final double armPeakOutputForward = 0.7;
    public static final int armAcceleration = 10000;
    public static final int armCruiseVelocity = 6000;

    public static final int armPositioningTolerance = 500; //maximum encoder count difference to be properly in a position
    public static final double armJoystickTiltToPositionFactor = 250;
    public static final int armCalibrationMotionIncrement = 300;

    //------------------------------------------------------------------------------------------------------------------
    //Aiming PID Constants for Vision Controlled Turning//
    //------------------------------------------------------------------------------------------------------------------
    public static Double aimingP;
    public static Double aimingI;
    public static Double aimingD;

    //
    //Climber Constants//
    //

    public static Integer climbMotorOneId;
    public static Integer climberDeploySolenoidId;
    public static Integer buddyClimbDeploySolenoidId;

    public static final double xBoxTriggerButtonThreshold = 0.5;
    public static final int motorControllerConfigTimeoutMs = 25;

    public static Double limelightMountingAngle;
    public static Double limelightMountingHeight;
    public static Double fieldVisionTargetHeight;
    public static Double fieldVisionDepthOfThreePointHoleFromVisionTarget;
    public static Double fieldVisionHeightOfThreePointHoleFromVisionTarget;
    public static double[] threePointHoleDistances;
    public static double[] threePointHoleAngles;
    public static double[] threePointHoleTopSpeeds;
    public static double[] threePointHoleBottomSpeeds;
    public static Integer driveLeftSlaveID2019_1;
    public static Integer driveLeftSlaveID2019_2;
    public static Integer driveRightSlaveID2019_1;
    public static Integer driveRightSlaveID2019_2;

    private static void frameSpecificConfig() {

        // Values specific to each physical robot

        switch (currentRobot) {
            case Alpha2020:
                isArmInstalled = true;
                isClimberInstalled = false;
                isCtrlPanelInstalled = false;
                isDriveInstalled = true;
                isIndexerInstalled = true;
                isIndexerSensorInstalled = false;
                isIntakeInstalled = true;
                isShooterInstalled = true;
                isVisionInstalled = false;
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

                topShooterMotorID = 2;
                bottomShooterMotorID = 10;
                topShooterPdpChannel = 12;
                bottomShooterPdpChannel = 13;

                shooterVelocityP = 0.04;
                shooterVelocityI = 0.0002;
                shooterVelocityIntegralZone = 500;
                shooterVelocityD = 0.0;
                shooterVelocityF = 0.05;

                intakeMotorID = 20;
                intakeMotorPdpChannel = 11;
                intakeSolenoidChannel1 = 1;
                intakeSolenoidChannel2 = 2;
                intakePistonExtendDelaySeconds = 1.0;
                intakePistonRetractDelaySeconds = 1.0;

                upperIndexerMotorID = 21;
                lowerIndexerMotorID = 22;
                indexerSensorID = 4;
                upperIndexerMotorPdpChannel = 8;
                lowerIndexerMotorPdpChannel = 9;
                indexerP = 0.1;
                indexerI = 0.0;
                indexerD = 0.0;
                indexerIntegralZone = 0;
                indexerF = 0.0;
                indexerPositioningTolerance = 200;
                powerCellDistanceInEncoderCounts = 5091;

                armMotorId = 3;
                armMotorPdpChannel = 3;

                armP = 0.1;
                armI = 3.54972071e-05;
                armIntegralZone = 3000;
                armD = 15.0;

                maxArmPosition = 180000;  // physical max = 190000
                trenchArmPosition = 45000;
                minArmPosition = 3000;
                armPositionIntakeStowedLimit = 45000;
                armPositionIntakeStowedTarget = armPositionIntakeStowedLimit + armPositioningTolerance;
                armPositionIntakeStowedUpperLimit = armPositionIntakeStowedTarget + armPositioningTolerance;
                armControlPanelPosition = 126000;

                IMUDriftConstant = -0.158;

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
                isIntakeInstalled = false;
                isShooterInstalled = false;
                isVisionInstalled = false;
                isPcmInstalled = false;
                isCompressorEnabled = false;
                isIMUInstalled = false;

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

                IMUDriftConstant = -0.158;

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
