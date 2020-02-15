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

    //------------------------------------------------------------------------------------------------------------------
    //Intake Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer IntakeMotorID;
    public static Integer IntakeMotorPdpChannel;
    public static Integer IntakeSolenoidChannel;

    public static Double IntakePistonExtendDelaySeconds;
    public static Double IntakePistonRetractDelaySeconds;

    public static final double intakeInwardPower = 0.6;
    public static final double intakeOutwardPower = 0.3;
    public static Boolean intakeDefaultIsRetracted;

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
    public static final int RotationControlSlices = 32;

    public static final double DeployDelaySeconds = .5;

    //Color Sensor thresholds-------------------------------------------------------------------------------------------
    public static final int ColorThreshold = 127;
    public static final int ColorEpsilon = 50;

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
    public static final double shooterIntakePowerTopMotor = 0.4;
    public static final double shooterIntakePowerBottomMotor = 0.4;

    public static final int shooterSpeedTolerance = 100; //Encoder counts per 100ms

    public static final double shooterLongRangeTopSpeed = 16000;
    public static final double shooterLongRangeBottomSpeed = 20000;
    public static final double shooterMidRangeTopSpeed = 8000;
    public static final double shooterMidRangeBottomSpeed = 14000;
    public static final double shooterCloseRangeBottomSpeed = 5000;
    public static final double shooterShortRangeBottomSpeed = 7000;

    //------------------------------------------------------------------------------------------------------------------
    //Indexer Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer PrimaryIndexerMotorID;
    public static Integer SecondaryIndexerMotorID;
    public static Integer IndexerPdpChannel;
    public static Double IndexerP;
    public static Double IndexerI;
    public static Integer IndexerIntegralZone;
    public static Double IndexerD;
    public static Double IndexerF;
    public static Integer StandardIndexerMotionInEncoderCounts;
    public static final Integer IndexerMaximumEncoderPositionRange = 20;
    public static Integer IndexerSensorID;

    //------------------------------------------------------------------------------------------------------------------
    //Balancer Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer BalancerMotorId;
    public static Integer BalancerPdpChannel;

    //------------------------------------------------------------------------------------------------------------------
    //Arm Constants//
    //------------------------------------------------------------------------------------------------------------------
    public static Integer armMotorId;
    public static Integer armHallEffectLimitSwitchId;
    public static Integer armMotorPdpChannel;

    // Arm positions MUST be overridden in frameSpecificConfig() when the arm is installed!
    // The values cannot be null when the arm isn't installed because they are used to initialize a static enum.
    public static int armPositionMaxValue = 0;
    public static int armPositionLongRangeValue = 0;
    public static int armPositionMidRangeValue = 0;
    public static int armPositionCloseRangeValue = 0;
    public static int armPositionTrenchValue = 0;
    public static int armPositionMinValue = 0;
    public static int armPositionHallEffectTopValue = 0;
    public static int armPositionIntakeStowedLimitValue = 0;

    // BEFORE SETTING armPIDTestMode TO TRUE:
    //   Bleed air.
    //   Manually extend the intake.
    //   Put the arm in the lowest physical position against the battery case.
    //
    // The encoder will be automatically zeroed after the code is deployed with this flag set true.
    // There is no need to change the default intake position because the compressor is disabled while
    // in tuning mode.
    public static final boolean armPIDTuningMode = false;

    // The procedure for running the arm without position sensors is similar to armPIDTestMode
    // and just as dangerous, but we suppress warnings in this mode because we're willing to
    // play with fire to make progress.
    //
    // BEFORE SETTING armNoPositionSensors TO TRUE:
    //   Connect intake pneumatic valve so intake is extended by default.
    //   Set intakeDefaultIsRetracted = false
    //   Inform all operators that ***BEFORE*** every power up of the robot, code push,
    //   roboRIO reboot or robot code restart, the following MUST be done:
    //     Manually extend the intake (if needed)
    //     Put the arm in the lowest physical position against the battery case
    //   If the arm is ever moved manually, the robot MUST be power cycled or the robot code MUST be restarted.
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
    public static final double armJoystickTiltToPositionFactor = 0.1;
    public static final int armCalibrationMotionIncrement = 3;

    //------------------------------------------------------------------------------------------------------------------
    //Aiming PID Constants for Vision Controlled Turning//
    //------------------------------------------------------------------------------------------------------------------
    public static Double AimingP;
    public static Double AimingI;
    public static Double AimingD;

    //
    //Climber Constants//
    //

    public static Integer ClimbMotorOneId;
    public static Integer ClimbMotorTwoId;
    public static Integer ClimberDeploySolenoidId;
    public static Integer HookDeploySolenoidId;
    public static Integer BuddyClimbDeploySolenoidId;


    public static final double XBoxTriggerButtonThreshold = 0.5;
    public static final int motorControllerConfigTimeoutMs = 10;

    private static void frameSpecificConfig() {

        // Values specific to each physical robot

        switch (currentRobot) {
            case Alpha2020:
                isArmInstalled = true;  //WARNING: MUST SET ARM POSITIONS BEFORE ENABLING ARM
                isClimberInstalled = false;
                isCtrlPanelInstalled = false;
                isDriveInstalled = true;
                isIndexerInstalled = false;
                isIntakeInstalled = true;
                isShooterInstalled = true;
                isVisionInstalled = false;
                isPcmInstalled = true;
                isCompressorEnabled = true;

                DriveLeftMasterID = 4;
                DriveLeftSlaveID = 16;
                DriveRightMasterID = 15;
                DriveRightSlaveID = 1;
                DriveLeftMasterPdpChannel = 1;
                DriveLeftSlavePdpChannel = 14;
                DriveRightMasterPdpChannel = 15;
                DriveRightSlavePdpChannel = 0;

                driveVelocityP = 0.02;
                driveVelocityI = 0.00015;
                driveVelocityIntegralZone = 250;
                driveVelocityD = 0.0006;
                driveVelocityF = 0.002;

                TopShooterMotorID = 10;
                BottomShooterMotorID = 2;
                TopShooterPdpChannel = 12;
                BottomShooterPdpChannel = 13;

                shooterVelocityP = 0.04;
                shooterVelocityI = 0.0002;
                shooterVelocityIntegralZone = 500;
                shooterVelocityD = 0.0;
                shooterVelocityF = 0.05;

                IntakeMotorID = 20;
                IntakeMotorPdpChannel = 11;
                IntakeSolenoidChannel = 3;
                IntakePistonExtendDelaySeconds = 1.0;
                IntakePistonRetractDelaySeconds = 1.0;
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

                armPositionMaxValue = 157983;
                armPositionLongRangeValue = 103000;
                armPositionMidRangeValue = 116436;
                armPositionCloseRangeValue = 138952;
                armPositionTrenchValue = 45000;
                armPositionMinValue = 3000;
                armPositionHallEffectTopValue = 0;
                armPositionIntakeStowedLimitValue = 45000;

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
