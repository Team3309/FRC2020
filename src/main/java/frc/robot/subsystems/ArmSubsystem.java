package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class ArmSubsystem extends SubsystemBase {

    public static final int MAXIMUM_ENCODER_DISTANCE_FOR_POSITION = 3; //maximum encoder distance to be properly in a position

    private boolean halifaxCalibrate;
    private DigitalInput halifaxLimitSwitch;
    public int desiredPosition;
    private boolean calibrated;

    private int initialEncoderCount;

    //we can't intialize certain states during the constructor because we really want to initialize them on first enable instead
    private boolean initialCalibration = true;
    private DigitalInput topLimitSwitch;
    private static final double CALIBRATION_VELOCITY = 0.01;
    private ArmPosition storedCalibratePosition;


    public enum ArmPosition {
        max(0),
        longRange(1),
        midRange(2),
        closeRange(3),
        trench(4),
        min(5),
        halifax(7),
        intermediate(6);

        int value;

        ArmPosition(int value) {
            this.value = value;
        }
    }

    private WPI_TalonFX armMotor;


    public ArmSubsystem() {
        calibrated = false;
        if (Config.isArmInstalled) {
            armMotor = new WPI_TalonFX(Config.ArmMotorId);
            initialEncoderCount = armMotor.getSelectedSensorPosition(0);
            halifaxLimitSwitch = new DigitalInput(Config.ArmHalifaxLimitSwitchId);
            topLimitSwitch = new DigitalInput((Config.ArmTopLimitSwitchId));

        }
    }

    /**
     * ---------------------------------------------------------------------------------------------------------------\
     * Moves arm based on a certain number of encoder counts.
     *
     * @param position - By how many encoder counts the arm should move.
     *
     */
    public void MoveArmManually(double position) {
        if (Config.isArmInstalled) {
            armMotor.set(ControlMode.Position, position);
        }
    }

    public boolean isInPosition() {
        if (!calibrated) {
            calibrate();
            return false;
        }
        return !Config.isArmInstalled || Math.abs(armMotor.getSelectedSensorPosition(0) - desiredPosition) < MAXIMUM_ENCODER_DISTANCE_FOR_POSITION;
    }


    private void calibrate() {
        if (Config.isArmInstalled) {
            //we want to have a one time only cycle because this stuff cannot be done before enable
            if(initialCalibration) {
                initialCalibration = false;
                halifaxCalibrate = halifaxLimitSwitch.get();
                armMotor.set(ControlMode.Velocity, CALIBRATION_VELOCITY);
            }
            if (halifaxCalibrate) {
                if (!halifaxLimitSwitch.get()) {
                    armMotor.set(ControlMode.PercentOutput, 0);
                    initialEncoderCount = armMotor.getSelectedSensorPosition(0);
                    calibrated = true;
                    if (storedCalibratePosition != null) {
                        this.MoveToPosition(storedCalibratePosition);
                    }
                }
            } else {
                if (topLimitSwitch.get()) {
                    armMotor.set(ControlMode.PercentOutput, 0);
                    initialEncoderCount = armMotor.getSelectedSensorPosition(0);
                    calibrated = true;
                    if (storedCalibratePosition != null) {
                        this.MoveToPosition(storedCalibratePosition);
                    }
                }
            }
        }
    }


    public boolean isHallifaxEngaged() {
        return !Config.isArmInstalled || halifaxLimitSwitch.get();
    }

    /**
     * ---------------------------------------------------------------------------------------------------------------\
     * Moves arm to a preset Arm position. Most of this is done for us properly with the PIDs and MotionMagic.
     *
     * @param position - The Arm position to which the arm will move.
     *
     */
    public void MoveToPosition(ArmPosition position) {
        if (Config.isArmInstalled) {
            if (calibrated) {
                armMotor.set(ControlMode.Position, armPositionToEncoderPosition(position));
            } else {
                storedCalibratePosition = position;
                calibrate();
            }

        }
    }

    /**---------------------------------------------------------------------------------------------------------------
     * converts an ArmPosition to the actually needed position for the encoder, accounting for initialization position
     * @param position ArmPosition to convert
     * @return The encoder position to feed to a motor
     */
    private double armPositionToEncoderPosition(ArmPosition position) {
        return position.value + initialEncoderCount - (halifaxCalibrate ? ArmPosition.halifax.value : ArmPosition.max.value);
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }
}
