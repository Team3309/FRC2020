package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.RobotContainer;


/**---------------------------------------------------------------------------------------------------------------------
 * @author Joshua Badzey
 *
 * The class for the power cell indexer, which will move power cells within the robot and determine how many
 * there are. It involves both the internal belt of the power cell containment area and the arm of the shooter.
 * Will work with power cell intake and shooter to determine how many power cells there are atany given moment.
 *
 */

public class IndexerSubsystem extends SubsystemBase {

    //put command protections inside subsystems (e.g: don't have conflicting commands)
    private WPI_TalonSRX upperIndexerMotor;
    private WPI_TalonSRX lowerIndexerMotor;
    private int upperMotorDesiredEncoderPosition;
    private int lowerMotorDesiredEncoderPosition;
    private DigitalInput PowerCellSensor;
    private int powerCells;
    private int lastCruiseVelocity = 0;

    public IndexerSubsystem() {
        if (Config.indexerAvailable) {
            upperIndexerMotor = new WPI_TalonSRX(Config.upperIndexerMotorID);
            lowerIndexerMotor = new WPI_TalonSRX(Config.lowerIndexerMotorID);
            if (Config.indexerSensorAvailable) {
                PowerCellSensor = new DigitalInput(Config.indexerSensorID);
            }
            configIndexerTalon(upperIndexerMotor);
            configIndexerTalon(lowerIndexerMotor);
            setIndexerSpeed(Config.indexInSpeed);  // configure motion magic parameters

            upperIndexerMotor.configNominalOutputForward(Config.indexerTopMinimumOutput, Config.motorControllerConfigTimeoutMs);
            upperIndexerMotor.configNominalOutputReverse(-Config.indexerTopMinimumOutput, Config.motorControllerConfigTimeoutMs);
            upperIndexerMotor.config_kP(0, Config.indexerTopPositionP, Config.motorControllerConfigTimeoutMs);
            upperIndexerMotor.config_kF(0, Config.indexerTopPositionF, Config.motorControllerConfigTimeoutMs);
            lowerIndexerMotor.configNominalOutputForward(Config.indexerBottomMinimumOutput, Config.motorControllerConfigTimeoutMs);
            lowerIndexerMotor.configNominalOutputReverse(-Config.indexerBottomMinimumOutput, Config.motorControllerConfigTimeoutMs);
            lowerIndexerMotor.config_kP(0, Config.indexerBottomPositionP, Config.motorControllerConfigTimeoutMs);
            lowerIndexerMotor.config_kF(0, Config.indexerBottomPositionF, Config.motorControllerConfigTimeoutMs);

            lowerIndexerMotor.setInverted(false);
            upperIndexerMotor.setInverted(true);

            lowerIndexerMotor.setSensorPhase(true);
            upperIndexerMotor.setSensorPhase(false);

            reset();
        }
    }

    private void configIndexerTalon(WPI_TalonSRX talon) {
        talon.configFactoryDefault();
        talon.configPeakOutputForward(Config.indexerPeakOutputForward, Config.motorControllerConfigTimeoutMs);
        talon.configPeakOutputReverse(Config.indexerPeakOutputReverse, Config.motorControllerConfigTimeoutMs);
        talon.configOpenloopRamp(Config.indexerOpenLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.configClosedloopRamp(Config.indexerClosedLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.config_kI(0, Config.indexerPositionI, Config.motorControllerConfigTimeoutMs);
        talon.config_IntegralZone(0, Config.indexerPositionIntegralZone, Config.motorControllerConfigTimeoutMs);
        talon.config_kD(0, Config.indexerPositionD, Config.motorControllerConfigTimeoutMs);
        talon.config_kP(1, Config.indexerVelocityP, Config.motorControllerConfigTimeoutMs);
        talon.config_kI(1, Config.indexerVelocityI, Config.motorControllerConfigTimeoutMs);
        talon.config_IntegralZone(1, Config.indexerVelocityIntegralZone, Config.motorControllerConfigTimeoutMs);
        talon.config_kD(1, Config.indexerVelocityD, Config.motorControllerConfigTimeoutMs);
        talon.config_kF(1, Config.indexerVelocityF, Config.motorControllerConfigTimeoutMs);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Change the max speed for the indexer motors to allow for the speed to vary for each firing solution.
     * The speed for intaking needs to be restored after firing. These settings are used for subsequent
     * MotionMagic commands sent to the motor controllers.
     *
     * @param cruiseVelocity encoder counts per 100ms
     */
    public void setIndexerSpeed(int cruiseVelocity) {
        if (Config.indexerAvailable && cruiseVelocity != lastCruiseVelocity) {
            // set motion magic parameters used for intake and position controlled shooting
            // these values are ignored for velocity controlled shooting
            upperIndexerMotor.configMotionCruiseVelocity(cruiseVelocity, Config.motorControllerConfigTimeoutMs);
            lowerIndexerMotor.configMotionCruiseVelocity(cruiseVelocity, Config.motorControllerConfigTimeoutMs);

            upperIndexerMotor.configMotionAcceleration(
                    (int) (cruiseVelocity / Config.indexerPositionRampSeconds), Config.motorControllerConfigTimeoutMs);
            lowerIndexerMotor.configMotionAcceleration(
                    (int) (cruiseVelocity / Config.indexerPositionRampSeconds), Config.motorControllerConfigTimeoutMs);
            lastCruiseVelocity = cruiseVelocity;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to pulse the belt forward by one power cell.
     *
     */
    public void indexOut() {
        if (Config.indexerAvailable && isInPosition()) {
            upperMotorDesiredEncoderPosition = upperIndexerMotor.getSelectedSensorPosition(0)
                    + Config.indexOutEncoderCounts[powerCells];
            lowerMotorDesiredEncoderPosition = lowerIndexerMotor.getSelectedSensorPosition(0)
                    + Config.indexOutEncoderCounts[powerCells];
            upperIndexerMotor.selectProfileSlot(0, 0);  // use position control PID parameters
            lowerIndexerMotor.selectProfileSlot(0, 0);
            upperIndexerMotor.set(ControlMode.MotionMagic, upperMotorDesiredEncoderPosition);
            lowerIndexerMotor.set(ControlMode.MotionMagic, lowerMotorDesiredEncoderPosition);
            decrementIndexerCounter();
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to pulse the belt backward by one power cell.
     *
     */
    public void indexIn() {
        if (Config.indexerAvailable && isInPosition()) {
            upperMotorDesiredEncoderPosition = upperIndexerMotor.getSelectedSensorPosition(0) -
                    Config.indexInEncoderCounts[powerCells];
            lowerMotorDesiredEncoderPosition = lowerIndexerMotor.getSelectedSensorPosition(0) -
                    Config.indexInEncoderCounts[powerCells];
            upperIndexerMotor.selectProfileSlot(0, 0);  // use position control PID parameters
            lowerIndexerMotor.selectProfileSlot(0, 0);
            upperIndexerMotor.set(ControlMode.MotionMagic, upperMotorDesiredEncoderPosition);
            lowerIndexerMotor.set(ControlMode.MotionMagic, lowerMotorDesiredEncoderPosition);
            incrementIndexerCounter();
        }
    }

    public void velocityShooting() {
        if (Config.indexerAvailable) {
            upperIndexerMotor.selectProfileSlot(1, 0);  // use velocity control PID parameters
            lowerIndexerMotor.selectProfileSlot(1, 0);
            upperIndexerMotor.set(ControlMode.Velocity, lastCruiseVelocity);
            lowerIndexerMotor.set(ControlMode.Velocity, lastCruiseVelocity);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Checks if the beam-break sensor is currently blocked.
     *
     * @return Whether the beam-break sensor is blocked.
     */
    public boolean isSensorBlocked() {
        if (Config.indexerSensorAvailable) {
            return !PowerCellSensor.get();
        }
        return false;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Increments the counter for the number of power cells currently in the indexer.
     *
     */
    public void incrementIndexerCounter() {
        if (powerCells < Config.maxPowerCells) {
            powerCells++;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Decrements the counter for the number of power cells currently in the indexer.
     *
     */
    public void decrementIndexerCounter() {
        if (powerCells > 0) {
            powerCells--;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Set the number of powercells externally to this subsystem, such as a known number of powercells preloaded
     * at the start of a match.
     *
     * @param powerCells Number of powercells currently loaded into the indexer
     */
    public void setNumPowerCells(int powerCells) {
        if (powerCells >= 0 && powerCells <= Config.maxPowerCells) {
            this.powerCells = powerCells;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Gets the number of power cells currently in the indexer.
     *
     * @return How many power cells are in the indexer.
     *
     */
    public int getCount() {
        return powerCells;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Checks the state of the beam-break sensor, and moves the indexer based on that state.
     *
     */
    public void autoIndexIn() {
        if (Config.indexerAvailable && Config.indexerSensorAvailable) {
            if (isSensorBlocked() && RobotContainer.getRobotState() == RobotContainer.RobotState.INTAKE) {
                indexIn();
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Checks whether the indexer has reached the target encoder position to within an acceptable range.
     * Don't check the top belt because it's less reliable due to the resistance from the extra pulleys.
     *
     * @return Whether the indexer is in position with an acceptable error.
     *
     */
    private boolean isInPosition() {
        return (Math.abs(lowerMotorDesiredEncoderPosition - lowerIndexerMotor.getSelectedSensorPosition())
                < Config.indexerPositioningTolerance);
    }

    public void reset() {
        if (Config.indexerAvailable) {
            // stop motors and cancel any pending motion magic movement demand if we are disabled
            upperIndexerMotor.set(ControlMode.PercentOutput, 0);
            lowerIndexerMotor.set(ControlMode.PercentOutput, 0);

            // reset desired encoder positions so there won't be any pending movement for the next index in/out operation
            upperMotorDesiredEncoderPosition = upperIndexerMotor.getSelectedSensorPosition(0);
            lowerMotorDesiredEncoderPosition = lowerIndexerMotor.getSelectedSensorPosition(0);
        }
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard.
     */
    public void outputToDashboard() {
        SmartDashboard.putNumber("Upper index desired position:", upperMotorDesiredEncoderPosition);
        SmartDashboard.putNumber("Upper index current position:",
                upperIndexerMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Upper index position error:",
                upperIndexerMotor.getSelectedSensorPosition(0) - upperMotorDesiredEncoderPosition);
        SmartDashboard.putNumber("Lower index desired position:", lowerMotorDesiredEncoderPosition);
        SmartDashboard.putNumber("Lower index current position:",
                lowerIndexerMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Lower index position error:",
                lowerIndexerMotor.getSelectedSensorPosition(0) - lowerMotorDesiredEncoderPosition);
        SmartDashboard.putNumber("Upper index velocity:", upperIndexerMotor.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Upper index velocity error:", upperIndexerMotor.getSelectedSensorVelocity(0) - lastCruiseVelocity);
        SmartDashboard.putNumber("Lower index velocity:", lowerIndexerMotor.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Lower index velocity error:", lowerIndexerMotor.getSelectedSensorVelocity(0) - lastCruiseVelocity);
        SmartDashboard.putNumber("Upper index power:", upperIndexerMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Lower index power:", lowerIndexerMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Upper index current:", Robot.pdp.getCurrent(Config.upperIndexerMotorPdpChannel));
        SmartDashboard.putNumber("Lower index current:", Robot.pdp.getCurrent(Config.lowerIndexerMotorPdpChannel));
        SmartDashboard.putBoolean("In position:", isInPosition());
        SmartDashboard.putNumber("Power Cell count:", getCount());
    }
}