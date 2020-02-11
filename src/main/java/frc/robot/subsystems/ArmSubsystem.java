package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {

    //------------------------------------------------------------------------------------------------------------------
    //Arm calibration variables//
    //------------------------------------------------------------------------------------------------------------------

    private boolean initialCalibration; //we can't intialize certain states during the constructor because we really want to initialize them on first enable instead
    private boolean halifaxCalibrate; //aka "quick calibrate" mode.
    private boolean calibrated; //whether or not the arm encoder is finised calibrating
    private int initialEncoderCount; //the encoder count at time of calibration finalization.
    private int desiredCalibrationPosition;
    private ArmPosition calibrationStoredPosition;

    private static final int CALIBRATION_MOTION_INCREMENT = 10;

    //------------------------------------------------------------------------------------------------------------------
    //Other arm variables//
    //------------------------------------------------------------------------------------------------------------------

    private static final int MAXIMUM_ENCODER_DISTANCE_FOR_POSITION = 3; //maximum encoder distance to be properly in a position
    private static final double JOYSTICK_TILT_TO_POSITION_ADJUSTMENT_CONVERSION_CONSTANT = 0.1;

    private int desiredPosition; //we can't actually store the ArmPosition because it's an enum and fine tuning / scan mode will forbid that.

    private DigitalInput topLimitSwitch;
    private DigitalInput halifaxLimitSwitch;

    private WPI_TalonFX armMotor;


    //TODO Find actual values for this part
    public enum ArmPosition {
        max(Config.ArmPositionMaxValue),
        longRange(Config.ArmPositionLongRangeValue),
        midRange(Config.ArmPositionMidRangeValue),
        closeRange(Config.ArmPositionCloseRangeValue),
        trench(Config.ArmPositionTrenchValue),
        min(Config.ArmPositionMinValue),
        halifaxTop(Config.ArmPositionHalifaxTopValue), //this is the highest position that the halifax switch will be engaged at.
        intermediate(0),
        intakeStowedLimit(Config.ArmPositionIntakeStowedLimitValue);
        int value;

        ArmPosition(int value) {
            this.value = value;
        }
    }



    public ArmSubsystem() {
        calibrated = false;
        initialCalibration = true;
        if (Config.isArmInstalled) {
            armMotor = new WPI_TalonFX(Config.ArmMotorId);
            //for now, just initialize to armMotor
            //TODO couldn't we just set the arm position before we get on the field using the halifax switch's static positioning
            armMotor.setNeutralMode(NeutralMode.Brake);
            initialEncoderCount = armMotor.getSelectedSensorPosition(0);
            try {
                //we try to enable, in case there is no halifax currently installed.
                halifaxLimitSwitch = new DigitalInput(Config.ArmHalifaxLimitSwitchId);
            } catch (Exception e) {
                DriverStation.reportWarning("ArmSubsystem: Halifax Limit Switch not found!", false);
            }
            topLimitSwitch = new DigitalInput((Config.ArmTopLimitSwitchId));

        }
    }

    /**
     * Adjust the arm in small amounts using speed control.
     * @param axisTilt a number between 0 and 1, generally describing the tilt of the joystick / trigger that is moving the trigger
     */
    public void adjustArm(double axisTilt) {
        if (Config.isArmInstalled) {
            //motion magic / position control doesn't work as well for fine tuning by the operator, who might want
            //to change by as much as just a couple encoder ticks, repeatedly and in quick succession.
            //So we use speed control.
            //TODO: alternatively, maybe use position control so they can make adjustments before the arm gets to its position in case they know the adjustments to make before hand.
            if (calibrated) {
                //so we don't attempt to crash the arm
                if (desiredPosition + axisTilt * JOYSTICK_TILT_TO_POSITION_ADJUSTMENT_CONVERSION_CONSTANT > armPositionToEncoderPosition(ArmPosition.intakeStowedLimit) &&
                        desiredPosition + axisTilt * JOYSTICK_TILT_TO_POSITION_ADJUSTMENT_CONVERSION_CONSTANT < armPositionToEncoderPosition(ArmPosition.max)) {
                    //DO NOT USE MOTION MAGIC. This is a small adjustment.
                    desiredPosition += axisTilt * JOYSTICK_TILT_TO_POSITION_ADJUSTMENT_CONVERSION_CONSTANT;
                    armMotor.set(ControlMode.Position, desiredPosition + axisTilt * JOYSTICK_TILT_TO_POSITION_ADJUSTMENT_CONVERSION_CONSTANT);
                }



            } else {
                calibrate();
            }
        }
    }


    /**----------------------------------------------------------------------------------------------------------------
     * Checks if the arm is in position. Also attempts to calibrate the arm if the arm is not calibrated yet.
     * We attempt to calibrate from here because it is the job of the subsystem and it is a method that
     * commands will repeatedly call as a finish condition for arm movement commands, allowing the subsystem
     * to check that calibration is complete.
     *
     * @return if the arm has reached its destination
     */
    public boolean isInPosition() {
        if (!calibrated) {
            calibrate();
            return false;
        }
        return !Config.isArmInstalled || (Math.abs(armMotor.getSelectedSensorPosition(0) - desiredPosition) < MAXIMUM_ENCODER_DISTANCE_FOR_POSITION);
    }

    /**----------------------------------------------------------------------------------------------------------------
     * Calibration method for the position of the arm motor encoder, this should take priority over moving to an arm position.
     */
    private void calibrate() {
        if (Config.isArmInstalled) {
            //we want to have a one time only cycle because this stuff cannot be done before enable
            if(initialCalibration) {
                initialCalibration = false;
                armMotor.setNeutralMode(NeutralMode.Brake);
                //in case the halifax is not installed, just use the top limit switch.
                //we check for switch engagement and not counterweight installation in case of slippage during coast or brake mode.
                //if there was slippage we want to not use the halifax calibration
                if (halifaxLimitSwitch != null) {
                    halifaxCalibrate = halifaxLimitSwitch.get();
                } else {
                    halifaxCalibrate = false;
                }
                desiredCalibrationPosition = armMotor.getSelectedSensorPosition(0) + CALIBRATION_MOTION_INCREMENT;
                armMotor.set(ControlMode.MotionMagic, desiredCalibrationPosition);

            }
            if (halifaxCalibrate) {
                if (!halifaxLimitSwitch.get()) {
                    armMotor.set(ControlMode.PercentOutput, 0);
                    initialEncoderCount = armMotor.getSelectedSensorPosition(0);
                    calibrated = true;
                    if (calibrationStoredPosition != null) {
                        this.moveToPosition(calibrationStoredPosition);

                    }
                }

            } else {
                //an optimization. This assumes that the halifax limit switch is engaged at a range of encoder positions, and not just one encoder position.
                //this optimization should still work on just one encoder position engaging the halifax switch, but has a very low likelihood of working.
                if (halifaxLimitSwitch != null) {
                    //in the case we have a halifax limit switch but no engagement at initialization, and obtain
                    //engagement in the middle of calibration (ie the arm slipped down and outside halifax range),
                    //we can swap to halifax initialization
                    //this is guaranteed to result in a faster calibration of the encoder than going all the way to the top
                    if (halifaxLimitSwitch.get()) {
                        //in this case we have triggered the lower threshold of the halifax limit switch
                        //switch to halifax calibration
                        halifaxCalibrate = true;
                        return; //we can immediately return because the halifax switch will need at least another cycle to complete.
                        //Although in theory we can record this position (assuming we know what it translates to in an arm position (READ: Extra tuning and work))
                        //We'd have to redo a large portion of the code to note three calibration final states to do so.
                    }
                }
                //of course if there is no halifax limit switch or the counterweight is not installed or the arm slipped up,
                //we need to just go to the top limit switch and start from there.
                if (topLimitSwitch.get()) {
                    armMotor.set(ControlMode.PercentOutput, 0);
                    initialEncoderCount = armMotor.getSelectedSensorPosition(0);
                    calibrated = true;
                    if (calibrationStoredPosition != null) {
                        this.moveToPosition(calibrationStoredPosition);
                    }
                }
            }
            //if we're close enough to the target point, set a new one.
            if ((Math.abs(armMotor.getSelectedSensorPosition(0) - desiredCalibrationPosition) < MAXIMUM_ENCODER_DISTANCE_FOR_POSITION)) {

                desiredCalibrationPosition += CALIBRATION_MOTION_INCREMENT;
                armMotor.set(ControlMode.MotionMagic, desiredCalibrationPosition);
            }
        }
    }

    /**
     * ---------------------------------------------------------------------------------------------------------------\
     * Moves arm to a preset Arm position. Most of this is done for us properly with the PIDs and MotionMagic.
     *
     * @param position - The Arm position to which the arm will move.
     *
     */
    public void moveToPosition(ArmPosition position) {
        if (Config.isArmInstalled) {
            if (calibrated) {

                desiredPosition = armPositionToEncoderPosition(position);
                armMotor.set(ControlMode.MotionMagic, armPositionToEncoderPosition(position));

            } else {
                calibrationStoredPosition = position;
                calibrate();
            }

        }
    }

    /**---------------------------------------------------------------------------------------------------------------
     * converts an ArmPosition to the actually needed position for the encoder, accounting for initialization position
     * @param position ArmPosition to convert
     * @return The encoder position to feed to a motor
     */
    private int armPositionToEncoderPosition(ArmPosition position) {
        return position.value + initialEncoderCount - (halifaxCalibrate ? ArmPosition.halifaxTop.value : ArmPosition.max.value);
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }
}
