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
    public static Boolean isPcmInstalled;
    public static Boolean isCompressorEnabled;

    //------------------------------------------------------------------------------------------------------------------
    //Drive Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Drive Motor IDs---------------------------------------------------------------------------------------------------
    public static Integer driveLeftMasterID;
    public static Integer driveLeftSlaveID;
    public static Integer driveRightMasterID;
    public static Integer driveRightSlaveID;

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
    public static final double driveWheelDiameterInInches = 6.0;
    public static final double encoderCountsPerDegree = 600;
    public static final double driveWheelRadiusInInches = driveWheelDiameterInInches /2;
    public static final double driveWheelInchesPerRevolution = driveWheelDiameterInInches * Math.PI;
    public static final double driveWheelEncoderCountsPerRevolution = encoderCountsPerDegree *
            360 * driveWheelRadiusInInches;

    //------------------------------------------------------------------------------------------------------------------
    //Intake Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer intakeMotorID;
    public static Integer intakeMotorPdpChannel;
    public static Integer intakeSolenoidChannel;

    public static Double intakePistonExtendDelaySeconds;
    public static Double intakePistonRetractDelaySeconds;

    public static final double intakeInwardPower = 0.6;
    public static final double intakeOutwardPower = 0.3;
    public static Boolean intakeDefaultIsRetracted;

    //------------------------------------------------------------------------------------------------------------------
    //Control Panel Manipulator Constants//
    //------------------------------------------------------------------------------------------------------------------
    //Manipulator Actuator IDs------------------------------------------------------------------------------------------
    public static Integer turnerMotorID;
    public static Integer turnerTractorPistonID; //Tractor: (retract = draw back, protract = draw forward, ergo tract=draw
    public static Integer turnerHeightAdjustmentPistonID;

    //Manipulator Actuator PDP Channels---------------------------------------------------------------------------------
    public static Integer turnerMotorPdpChannel;

    //Manipulator Physical Constants------------------------------------------------------------------------------------
    public static final double turnerWheelRadiusInInches = 4.0;  //  value?
    public static final double turnerWheelDiameterInInches = turnerWheelRadiusInInches *2;
    public static final double turnerWheelInchesPerRevolution = turnerWheelDiameterInInches * Math.PI;

    //Manipulator Control Constants-------------------------------------------------------------------------------------
    public static final double turnerRotationPower = 0.4;
    public static final int rotationControlSlices = 32;

    public static final double deployDelayInSeconds = .5;

    //Color Sensor thresholds-------------------------------------------------------------------------------------------
    public static final int colorThreshold = 127;
    public static final int colorEpsilon = 50;

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
    public static final double shooterClosedLoopRampRate = 1.0;
    public static final double shooterOpenLoopRampRate = 1.0;
    public static Double shooterVelocityP;
    public static Double shooterVelocityI;
    public static Integer shooterVelocityIntegralZone;
    public static Double shooterVelocityD;
    public static Double shooterVelocityF;
    public static final double shooterIntakePowerTopMotor = 0.4;
    public static final double shooterIntakePowerBottomMotor = 0.4;

    public static final int shooterSpeedTolerance = 100; //Encoder counts per 100ms

    public static final double shooterLongRangeTopSpeed = 16250;
    public static final double shooterLongRangeBottomSpeed = 20000;
    public static final double shooterMidRangeTopSpeed = 16250;
    public static final double shooterMidRangeBottomSpeed = 20000;
    public static final double shooterCloseRangeBottomSpeed = 16250;
    public static final double shooterShortRangeBottomSpeed = 20000;

    //------------------------------------------------------------------------------------------------------------------
    //Indexer Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer upperIndexerMotorID;
    public static Integer lowerIndexerMotorID;
    public static Integer upperIndexerMotorPdpChannel;
    public static Integer lowerIndexerMotorPdpChannel;
    public static Double indexerOpenLoopRampRate;
    public static Double indexerClosedLoopRampRate;
    public static Double indexerP;
    public static Double indexerI;
    public static Integer indexerIntegralZone;
    public static Double indexerD;
    public static Double indexerF;
    public static Integer powerCellDistanceInEncoderCounts;
    public static final Integer indexerPositioningTolerance = 20;
    public static Integer indexerSensorID;
    public static int maxPowerCells = 5;

    //------------------------------------------------------------------------------------------------------------------
    //Balancer Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer balancerMotorId;
    public static Integer balancerPdpChannel;

    //------------------------------------------------------------------------------------------------------------------
    //Arm Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer armMotorId;
    public static Integer armHallEffectLimitSwitchId;
    public static Integer armMotorPdpChannel;

    // Arm positions MUST be overridden in frameSpecificConfig() when the arm is installed!
    // The values cannot be null when the arm isn't installed because they are used to initialize a static enum.
    public static int maxArmPosition = 0;
    public static int longRangeArmPosition = 0;
    public static int midRangeArmPosition = 0;
    public static int closeRangeArmPosition = 0;
    public static int trenchArmPosition = 0;
    public static int minArmPosition = 0;
    public static int armPositionHallEffectTopValue = 0;
    public static int armPositionIntakeStowedLimitValue = 0;
    public static int armPositionIntakeStowedUpperLimit = 0;

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
    //   Connect intake pneumatic valve so intake is extended by default.
    //   Set intakeDefaultIsRetracted = false
    //   Inform all operators that ***BEFORE*** every power up of the robot, the following MUST be done:
    //     Manually extend the intake.
    //     Put the arm in the lowest physical position against the battery case.
    public static final boolean armNoPositionSensors = true;

    public static Double armP;
    public static Double armI;
    public static Integer armIntegralZone;
    public static Double armD;

    public static final double peakOutputReverse = -0.2;
    public static final double peakOutputForward = 0.7;
    public static final int armAcceleration = 10000;
    public static final int armCruiseVelocity = 6000;

    public static final int armPositioningTolerance = 500; //maximum encoder count difference to be properly in a position
    public static final double armJoystickTiltToPositionFactor = 250;
    public static final int armCalibrationMotionIncrement = 3;

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
    public static Integer climbMotorTwoId;
    public static Integer climberDeploySolenoidId;
    public static Integer hookDeploySolenoidId;
    public static Integer buddyClimbDeploySolenoidId;


    public static final double xBoxTriggerButtonThreshold = 0.5;
    public static final int motorControllerConfigTimeoutMs = 25;

    private static void frameSpecificConfig() {

        // Values specific to each physical robot

        switch (currentRobot) {
            case Alpha2020:
                isArmInstalled = true;  //WARNING: MUST SET ARM POSITIONS BEFORE ENABLING ARM
                isClimberInstalled = false;
                isCtrlPanelInstalled = false;
                isDriveInstalled = true;
                isIndexerInstalled = true;
                isIntakeInstalled = true;
                isShooterInstalled = true;
                isVisionInstalled = false;
                isPcmInstalled = true;
                isCompressorEnabled = true;

                driveLeftMasterID = 4;
                driveLeftSlaveID = 16;
                driveRightMasterID = 15;
                driveRightSlaveID = 1;
                driveLeftMasterPdpChannel = 1;
                driveLeftSlavePdpChannel = 14;
                driveRightMasterPdpChannel = 15;
                driveRightSlavePdpChannel = 0;

                driveVelocityP = 0.02;
                driveVelocityI = 0.00015;
                driveVelocityIntegralZone = 250;
                driveVelocityD = 0.0006;
                driveVelocityF = 0.002;

                topShooterMotorID = 10;
                bottomShooterMotorID = 2;
                topShooterPdpChannel = 12;
                bottomShooterPdpChannel = 13;

                shooterVelocityP = 0.04;
                shooterVelocityI = 0.0002;
                shooterVelocityIntegralZone = 500;
                shooterVelocityD = 0.0;
                shooterVelocityF = 0.05;

                intakeMotorID = 20;
                intakeMotorPdpChannel = 11;
                intakeSolenoidChannel = 3;
                intakePistonExtendDelaySeconds = 1.0;
                intakePistonRetractDelaySeconds = 1.0;
                intakeDefaultIsRetracted = false;  // should be true for competition

                armMotorId = 3;
                armMotorPdpChannel = 3;

                armP = 0.1;
                armI = 3.54972071e-05;
                armIntegralZone = 3000;
                armD = 15.0;

                // Physical max with chain tight on bottom = 187676
                // Physical max with chain tight on top = 181690
                // Safe max with chain tight on bottom = 157983 (stays in place)
                // Close shot with chain tight on bottom = 138952 (stays in place)
                // Mid shot with chain tight on bottom = 116436 (drops slowly)
                // Far shot with chain tight on bottom = 99568 (drops fast)
                // Trench drive with chain tight on bottom = 47955 (drops fast)
                // Above intake with chain tight on bottom = 41538 (drops fast)
                // Above intake with chain tight on top = 32914 (drops fast)
                // On battery case with chain tight on bottom super light drop < 3000 (blocked)
                // On battery case with chain tight on bottom hard drop > -7000 (blocked)

                maxArmPosition = 157983;
                longRangeArmPosition = 103000; //103000
                midRangeArmPosition = 100000; //116436;
                closeRangeArmPosition = 97000; //138952;
                trenchArmPosition = 45000;
                minArmPosition = 3000;
                armPositionHallEffectTopValue = 0;
                armPositionIntakeStowedLimitValue = 45000;
                armPositionIntakeStowedUpperLimit = 500;

                break;

            case Practice2017:
                isArmInstalled = false;  //WARNING: MUST SET ARM POSITIONS BEFORE ENABLING ARM
                isClimberInstalled = false;
                isCtrlPanelInstalled = false;
                isDriveInstalled = true;
                isIndexerInstalled = false;
                isIntakeInstalled = false;
                isShooterInstalled = false;
                isVisionInstalled = false;
                isPcmInstalled = false;
                isCompressorEnabled = false;

                driveLeftMasterID = 6;
                driveLeftSlaveID = 8;
                driveRightMasterID = 5;
                driveRightSlaveID = 7;
                driveLeftMasterPdpChannel = 13;
                driveLeftSlavePdpChannel = 14;
                driveRightMasterPdpChannel = 1;
                driveRightSlavePdpChannel = 2;

                driveVelocityP = 0.1;
                driveVelocityI = 0.0;
                driveVelocityIntegralZone = 1000;
                driveVelocityD = 0.0;
                driveVelocityF = 0.0;
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
