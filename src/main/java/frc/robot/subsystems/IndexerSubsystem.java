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
    private WPI_TalonSRX UpperIndexerMotor;
    private WPI_TalonSRX LowerIndexerMotor;
    private int UpperMotorDesiredEncoderPosition;
    private int LowerMotorDesiredEncoderPosition;
    private DigitalInput PowerCellSensor;
    private int PowerCells;
    private int lastIndexerSpeed = 0;

    public IndexerSubsystem() {
        if (Config.isIndexerInstalled) {
            UpperIndexerMotor = new WPI_TalonSRX(Config.upperIndexerMotorID);
            LowerIndexerMotor = new WPI_TalonSRX(Config.lowerIndexerMotorID);
            if (Config.isIndexerSensorInstalled) {
                PowerCellSensor = new DigitalInput(Config.indexerSensorID);
            }
            configIndexerTalon(UpperIndexerMotor);
            configIndexerTalon(LowerIndexerMotor);
            setIndexerSpeed(Config.indexInSpeed);

            LowerIndexerMotor.setInverted(false);
            UpperIndexerMotor.setInverted(true);

            LowerIndexerMotor.setSensorPhase(true);
            UpperIndexerMotor.setSensorPhase(false);

            reset();
        }
    }

    private void configIndexerTalon(WPI_TalonSRX talon) {

        talon.configFactoryDefault();

        talon.configOpenloopRamp(Config.indexerOpenLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.configClosedloopRamp(Config.indexerClosedLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.config_kP(0, Config.indexerP, Config.motorControllerConfigTimeoutMs);
        talon.config_kI(0, Config.indexerI, Config.motorControllerConfigTimeoutMs);
        talon.config_IntegralZone(0, Config.indexerIntegralZone, Config.motorControllerConfigTimeoutMs);
        talon.config_kD(0, Config.indexerD, Config.motorControllerConfigTimeoutMs);
        talon.config_kF(0, Config.indexerF, Config.motorControllerConfigTimeoutMs);

        // Motion Magic parameters
        talon.configPeakOutputForward(Config.indexerPeakOutputForward, Config.motorControllerConfigTimeoutMs);
        talon.configPeakOutputReverse(Config.indexerPeakOutputReverse, Config.motorControllerConfigTimeoutMs);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Change the max speed for the indexer motors to allow for the speed to vary for each firing solution.
     * The speed for intaking needs to be restored after firing. These settings are used for subsequent
     * MotionMagic commands sent to the motor controllers.
     *
     * @param cruiseVelocity encoder counts per 100ms
     */
    public void setIndexerSpeed(int cruiseVelocity) {
        if (Config.isIndexerInstalled && cruiseVelocity != lastIndexerSpeed) {
            UpperIndexerMotor.configMotionCruiseVelocity(cruiseVelocity, Config.motorControllerConfigTimeoutMs);
            LowerIndexerMotor.configMotionCruiseVelocity(cruiseVelocity, Config.motorControllerConfigTimeoutMs);

            UpperIndexerMotor.configMotionAcceleration(
                    (int) (cruiseVelocity / Config.indexerRampSeconds), Config.motorControllerConfigTimeoutMs);
            LowerIndexerMotor.configMotionAcceleration(
                    (int) (cruiseVelocity / Config.indexerRampSeconds), Config.motorControllerConfigTimeoutMs);
            lastIndexerSpeed = cruiseVelocity;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to pulse the belt forward by one power cell.
     *
     */
    public void indexOut() {
        if (Config.isIndexerInstalled) {
            if (isInPosition()) {
                UpperMotorDesiredEncoderPosition = UpperIndexerMotor.getSelectedSensorPosition(0)
                        + Config.indexOutEncoderCounts[PowerCells];
                LowerMotorDesiredEncoderPosition = LowerIndexerMotor.getSelectedSensorPosition(0)
                        + Config.indexOutEncoderCounts[PowerCells];
                UpperIndexerMotor.set(ControlMode.MotionMagic, UpperMotorDesiredEncoderPosition);
                LowerIndexerMotor.set(ControlMode.MotionMagic, LowerMotorDesiredEncoderPosition);
                decrementIndexerCounter();
            }
        }
    }

    public void velocityShooting() {
        if (Config.isIndexerInstalled) {
            UpperIndexerMotor.set(ControlMode.Velocity, lastIndexerSpeed);
            LowerIndexerMotor.set(ControlMode.Velocity, lastIndexerSpeed);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to pulse the belt backward by one power cell.
     *
     */
    public void indexIn() {
        if (Config.isIndexerInstalled) {
//            if (isInPosition()) {
                UpperMotorDesiredEncoderPosition = UpperIndexerMotor.getSelectedSensorPosition(0) -
                        Config.indexInEncoderCounts[PowerCells];
                LowerMotorDesiredEncoderPosition = LowerIndexerMotor.getSelectedSensorPosition(0) -
                        Config.indexInEncoderCounts[PowerCells];
                UpperIndexerMotor.set(ControlMode.MotionMagic, UpperMotorDesiredEncoderPosition);
                LowerIndexerMotor.set(ControlMode.MotionMagic, LowerMotorDesiredEncoderPosition);
                incrementIndexerCounter();
//            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Checks if the beam-break sensor is currently blocked.
     *
     * @return Whether the beam-break sensor is blocked.
     */
    public boolean isSensorBlocked() {
        if (Config.isIndexerSensorInstalled) {
            return !PowerCellSensor.get();
        }
        return false;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Increments the counter for the number of power cells currently in the indexer.
     *
     */
    public void incrementIndexerCounter() {
        if (PowerCells < Config.maxPowerCells) {
            PowerCells++;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Decrements the counter for the number of power cells currently in the indexer.
     *
     */
    public void decrementIndexerCounter() {
        if (PowerCells > 0) {

            PowerCells--;
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
            this.PowerCells = powerCells;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Gets the number of power cells currently in the indexer.
     *
     * @return How many power cells are in the indexer.
     *
     */
    public int getCount() {
        return PowerCells;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Checks the state of the beam-break sensor, and moves the indexer based on that state.
     *
     */
    public void autoIndexIn() {
        if (Config.isIndexerInstalled && Config.isIndexerSensorInstalled) {
            if (isSensorBlocked() && RobotContainer.getRobotState() == RobotContainer.RobotState.INTAKE) {
                indexIn();
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Checks whether the indexer has reached the target encoder position to within an acceptable range.
     *
     * @return Whether the indexer is in position with an acceptable error.
     *
     */
    private boolean isInPosition() {
        return ((Math.abs(UpperMotorDesiredEncoderPosition - UpperIndexerMotor.getSelectedSensorPosition())
                < Config.indexerPositioningTolerance)) && (Math.abs(LowerMotorDesiredEncoderPosition
                - LowerIndexerMotor.getSelectedSensorPosition()) < Config.indexerPositioningTolerance);
    }

    public void reset() {
        if (Config.isIndexerInstalled) {
            // stop motors and cancel any pending motion magic movement demand if we are disabled
            UpperIndexerMotor.set(ControlMode.PercentOutput, 0);
            LowerIndexerMotor.set(ControlMode.PercentOutput, 0);

            // reset desired encoder positions so there won't be any pending movement for the next index in/out operation
            UpperMotorDesiredEncoderPosition = UpperIndexerMotor.getSelectedSensorPosition(0);
            LowerMotorDesiredEncoderPosition = LowerIndexerMotor.getSelectedSensorPosition(0);
        }
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard.
     */
    public void outputToDashboard() {
        SmartDashboard.putNumber("Upper motor desired encoder position:", UpperMotorDesiredEncoderPosition);
        SmartDashboard.putNumber("Upper motor current encoder position:",
                UpperIndexerMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Lower motor desired encoder position:", LowerMotorDesiredEncoderPosition);
        SmartDashboard.putNumber("Lower motor current encoder position:",
                LowerIndexerMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Upper motor power:", UpperIndexerMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Lower motor power:", LowerIndexerMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Upper motor current:", Robot.pdp.getCurrent(Config.upperIndexerMotorPdpChannel));
        SmartDashboard.putNumber("Lower motor current:", Robot.pdp.getCurrent(Config.lowerIndexerMotorPdpChannel));
        SmartDashboard.putBoolean("In position:", isInPosition());
        SmartDashboard.putBoolean("Sensor blocked:", isSensorBlocked());
        SmartDashboard.putNumber("Power Cell count:", getCount());
    }
}