package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;

import static com.ctre.phoenix.motorcontrol.LimitSwitchNormal.NormallyClosed;
import static com.ctre.phoenix.motorcontrol.LimitSwitchSource.FeedbackConnector;

public class ArmSubsystem extends SubsystemBase {

    //------------------------------------------------------------------------------------------------------------------
    //Arm calibration variables//
    //------------------------------------------------------------------------------------------------------------------

    private boolean initialCalibration; //we can't initialize certain states during the constructor because we really want to initialize them on first enable instead
    private boolean hallEffectCalibrate; //aka "quick calibrate" mode.
    private boolean calibrated; //whether or not the arm encoder is finished calibrating
    private int initialEncoderCount; //the encoder count at time of calibration finalization, relative to its initialization position.
    private int desiredCalibrationPosition;
    private Integer calibrationStoredPosition;

    //------------------------------------------------------------------------------------------------------------------
    //Other arm variables//
    //------------------------------------------------------------------------------------------------------------------

    private int desiredPosition; //we can't actually store the ArmPosition because it's an enum and fine tuning / scan mode will forbid that.

    private DigitalInput hallEffectLimitSwitch;

    private WPI_TalonFX armMotor;

    public boolean isArmAboveIntakeMinimum() {
        if (!Config.isArmInstalled) {
            return true;
        }
        return armMotor.getSelectedSensorPosition(0) >= armPositionToEncoderPosition(Config.armPositionIntakeStowedLimit);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Constructor. All methods that move the arm require the arm to be installed.
     *
     */
    public ArmSubsystem() {
        calibrated = false;
        initialCalibration = true;
        if (Config.isArmInstalled) {
            armMotor = new WPI_TalonFX(Config.armMotorId);
            configTalon(armMotor);
            initialEncoderCount = armMotor.getSelectedSensorPosition(0);
            try {
                //we try to enable, in case there is no hall effect currently installed.
                hallEffectLimitSwitch = new DigitalInput(Config.armHallEffectLimitSwitchId);
            } catch (Exception e) {
                DriverStation.reportWarning("ArmSubsystem: Hall Effect Limit Switch not found!", false);
            }

        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Configures arm motor and limit switches.
     *
     * @param talon The motor to be configured.
     *
     */
    private void configTalon(WPI_TalonFX talon) {
        talon.configFactoryDefault();
        talon.setSensorPhase(false);
        talon.setInverted(true);
        talon.configForwardLimitSwitchSource(FeedbackConnector, NormallyClosed, Config.motorControllerConfigTimeoutMs);
        talon.setNeutralMode(NeutralMode.Brake);

        // Position control PID parameters
        talon.config_kP(0, Config.armP, Config.motorControllerConfigTimeoutMs);
        talon.config_kI(0, Config.armI, Config.motorControllerConfigTimeoutMs);
        talon.config_IntegralZone(0, Config.armIntegralZone, Config.motorControllerConfigTimeoutMs);
        talon.config_kD(0, Config.armD, Config.motorControllerConfigTimeoutMs);

        // Motion Magic parameters
        talon.configMotionCruiseVelocity(Config.armCruiseVelocity, Config.motorControllerConfigTimeoutMs);
        talon.configMotionAcceleration(Config.armAcceleration, Config.motorControllerConfigTimeoutMs);
        talon.configPeakOutputForward(Config.armPeakOutputForward, Config.motorControllerConfigTimeoutMs);
        talon.configPeakOutputReverse(Config.armPeakOutputReverse, Config.motorControllerConfigTimeoutMs);

        // TODO: Is this useful or just extra overhead?
        addChild("Arm Motor", talon);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Sets the neutral mode for the arm motor to Brake (immediate stop).
     *
     */
    public void setBrakeMode() {
        if (Config.isArmInstalled) {
            armMotor.setNeutralMode(NeutralMode.Brake);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Sets the neutral mode for the arm motor to Coast (gradual stop).
     *
     */
    public void setCoastMode() {
        if (Config.isArmInstalled) {
            armMotor.setNeutralMode(NeutralMode.Coast);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Only for use when disabled to cancel previous goal position.
     *
     */
    public void stopMotor() {
        if (Config.isArmInstalled) {
            armMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Adjust the arm in small amounts using speed control.
     * @param axisTilt a number between 0 and 1, generally describing the tilt of the joystick / trigger
     *                 that is moving the trigger
     */
    public void adjustArm(double axisTilt) {
        if (Config.isArmInstalled) {
            //motion magic / position control doesn't work as well for fine tuning by the operator, who might want
            //to change by as much as just a couple encoder ticks, repeatedly and in quick succession.
            //So we use speed control.
            if (calibrated) {
                int newDesiredPosition = (int) (desiredPosition + axisTilt * Config.armJoystickTiltToPositionFactor);
                //so we don't attempt to crash the arm
                if (newDesiredPosition > armPositionToEncoderPosition(Config.armPositionIntakeStowedLimit) &&
                        newDesiredPosition + axisTilt * Config.armJoystickTiltToPositionFactor < armPositionToEncoderPosition(Config.maxArmPosition)) {
                    //DO NOT USE MOTION MAGIC. This is a small adjustment.
                    desiredPosition = newDesiredPosition;
                    armMotor.set(ControlMode.Position, newDesiredPosition);
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
     * @return true if the arm has reached its destination or if no arm is installed.
     *
     */
    public boolean isInPosition() {
        if (!Config.isArmInstalled) {
            return true;
        }
        if (!calibrated) {
            calibrate();
            return false;
        }
        return (Math.abs(armMotor.getSelectedSensorPosition(0) - desiredPosition) < Config.armPositioningTolerance);
    }

    /**----------------------------------------------------------------------------------------------------------------
     * Calibration method for the position of the arm motor encoder, this should take priority over
     * moving to an arm position.
     *
     */
    private void calibrate() {
        if (Config.isArmInstalled) {
            if (Config.armPIDTuningMode || Config.armNoPositionSensors) {
                initialEncoderCount = Config.armPositionHardStop; // Arm was required to be at the hard stop at start up.
                initialCalibration = false;
                calibrated = true;
                if (calibrationStoredPosition != null) {
                    moveToPosition(calibrationStoredPosition);
                }
                return;
            }
            //we want to have a one time only cycle because this stuff cannot be done before enable
            if(initialCalibration) {
                initialCalibration = false;
                //in case the hall effect is not installed, just use the top limit switch.
                //we check for switch engagement and not counterweight installation in case of slippage during coast or brake mode.
                //if there was slippage we want to not use the hall effect calibration
                if (hallEffectLimitSwitch != null) {
                    hallEffectCalibrate = hallEffectLimitSwitch.get();
                } else {
                    hallEffectCalibrate = false;
                }
                desiredCalibrationPosition = armMotor.getSelectedSensorPosition(0) +
                        Config.armCalibrationMotionIncrement;
                armMotor.set(ControlMode.MotionMagic, desiredCalibrationPosition);
            }
            if (hallEffectCalibrate) {
                if (!hallEffectLimitSwitch.get()) {
                    armMotor.set(ControlMode.PercentOutput, 0);
                    initialEncoderCount = armMotor.getSelectedSensorPosition(0) - Config.armPositionHallEffectTop;
                    calibrated = true;
                    if (calibrationStoredPosition != null) {
                        this.moveToPosition(calibrationStoredPosition);
                    }
                }

            } else {
                //an optimization. This assumes that the hall effect limit switch is engaged at a range of encoder positions, and not just one encoder position.
                //this optimization should still work on just one encoder position engaging the hall effect switch, but has a very low likelihood of working.
                if (hallEffectLimitSwitch != null) {
                    //in the case we have a hall effect limit switch but no engagement at initialization, and obtain
                    //engagement in the middle of calibration (ie the arm slipped down and outside hall effect range),
                    //we can swap to hall effect initialization
                    //this is guaranteed to result in a faster calibration of the encoder than going all the way to the top
                    if (hallEffectLimitSwitch.get()) {
                        //in this case we have triggered the lower threshold of the hall effect limit switch
                        //switch to hall effect calibration
                        hallEffectCalibrate = true;
                        return; //we can immediately return because the hall effect switch will need at least another cycle to complete.
                        //Although in theory we can record this position (assuming we know what it translates to in an arm position (READ: Extra tuning and work))
                        //We'd have to redo a large portion of the code to note three calibration final states to do so.
                    }
                }
                //of course if there is no hall effect limit switch or the counterweight is not installed or the arm slipped up,
                //we need to just go to the top limit switch and start from there.
                if (isArmAtUpperLimit()) {
                    armMotor.set(ControlMode.PercentOutput, 0);
                    initialEncoderCount = armMotor.getSelectedSensorPosition(0) - Config.maxArmPosition;
                    calibrated = true;
                    if (calibrationStoredPosition != null) {
                        moveToPosition(calibrationStoredPosition);
                    }
                }
            }
            //if we're close enough to the target point, set a new one.
            if ((Math.abs(armMotor.getSelectedSensorPosition(0) - desiredCalibrationPosition) <
                    Config.armPositioningTolerance)) {
                desiredCalibrationPosition += Config.armCalibrationMotionIncrement;
                armMotor.set(ControlMode.MotionMagic, desiredCalibrationPosition);
            }
        }
    }

    /**---------------------------------------------------------------------------------------------------------------\
     * Moves arm to a preset Arm position. Most of this is done for us properly with the PIDs and MotionMagic.
     *
     * @param position - The Arm position to which the arm will move.
     *
     */
    public void moveToPosition(int position) {
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
     * Converts an ArmPosition to the actually needed position for the encoder, accounting for initialization position
     * @param position ArmPosition to convert
     * @return The encoder position to feed to a motor
     *
     */
    private int armPositionToEncoderPosition(int position) {
        return position + initialEncoderCount;
    }

    /**---------------------------------------------------------------------------------------------------------------
     * Detects if the arm is physically triggering the upper limit switch
     * @return true if the limit switch is engaged
     *
     */
    private boolean isArmAtUpperLimit() {
        return armMotor.isFwdLimitSwitchClosed() == 0;
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     *
     */
    public void outputToDashboard() {
        SmartDashboard.putNumber("Arm encoder position", armMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Arm desired encoder position", desiredPosition);
        SmartDashboard.putNumber("Arm power", armMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Arm current", Robot.pdp.getCurrent(Config.armMotorPdpChannel));
        SmartDashboard.putBoolean("Arm upper limit switch", isArmAtUpperLimit());
        SmartDashboard.putBoolean("Arm calibrated", calibrated);
    }
}
