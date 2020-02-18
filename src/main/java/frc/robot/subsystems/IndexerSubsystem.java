package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
    public enum IndexerState {
        OFF,
        INDEXING_IN,
        INDEXING_OUT
    }

    private IndexerState indexerState = IndexerState.OFF;
    private WPI_TalonSRX UpperIndexerMotor;
    private WPI_TalonSRX LowerIndexerMotor;
    private int UpperMotorDesiredEncoderPosition;
    private int LowerMotorDesiredEncoderPosition;
    private DigitalInput PowerCellSensor;
    private int PowerCells;
    private boolean isFinishedIndexing = true;

    public IndexerSubsystem() {
        if (Config.isIndexerInstalled) {
            UpperIndexerMotor = new WPI_TalonSRX(Config.upperIndexerMotorID);
            UpperMotorDesiredEncoderPosition = UpperIndexerMotor.getSelectedSensorPosition(0);
            LowerIndexerMotor = new WPI_TalonSRX(Config.lowerIndexerMotorID);
            if (Config.isIndexerSensorInstalled) {
                PowerCellSensor = new DigitalInput(Config.indexerSensorID);
            }
            configIndexerTalon(UpperIndexerMotor);
            configIndexerTalon(LowerIndexerMotor);
        }
    }

    private void configIndexerTalon(WPI_TalonSRX talon) {

        talon.configFactoryDefault();
        talon.setInverted(true);
        talon.setSensorPhase(true);

        talon.configOpenloopRamp(Config.indexerOpenLoopRampRate);
        talon.configClosedloopRamp(Config.indexerClosedLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.config_kP(0, Config.indexerP, Config.motorControllerConfigTimeoutMs);
        talon.config_kI(0, Config.indexerI, Config.motorControllerConfigTimeoutMs);
        talon.config_IntegralZone(0, Config.indexerIntegralZone, Config.motorControllerConfigTimeoutMs);
        talon.config_kD(0, Config.indexerD, Config.motorControllerConfigTimeoutMs);
        talon.config_kF(0, Config.indexerF, Config.motorControllerConfigTimeoutMs);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to pulse the belt forward by one power cell.
     *
     */
    public void indexOut() {
        if (Config.isIndexerInstalled) {
            //check if the indexer is indexing out and if it is currently free for operation
            if(indexerState == IndexerState.INDEXING_OUT && isFinishedIndexing)
                //indexer is currently being used.
                isFinishedIndexing = false;
                UpperMotorDesiredEncoderPosition = UpperIndexerMotor.getSelectedSensorPosition(0)
                        - Config.powerCellDistanceInEncoderCounts;
                LowerMotorDesiredEncoderPosition = LowerIndexerMotor.getSelectedSensorPosition(0)
                        - Config.powerCellDistanceInEncoderCounts;
                UpperIndexerMotor.set(ControlMode.MotionMagic, UpperMotorDesiredEncoderPosition);
                LowerIndexerMotor.set(ControlMode.MotionMagic, LowerMotorDesiredEncoderPosition);
                decrementIndexerCounter();
                //indexer has done its job, so it can deactivate.
                indexerState = IndexerState.OFF;
                //the indexer is free for use by commands.
                isFinishedIndexing = true;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to pulse the belt backward by one power cell.
     *
     */
    public void indexIn() {
        if (Config.isIndexerInstalled) {
            if (isInPosition()) {
                //checks if indexer is indexing in and if it is free for use by commands.
                if (indexerState == IndexerState.INDEXING_IN && isFinishedIndexing) {
                    //indexer is currently being used by a command.
                    isFinishedIndexing = false;
                    UpperMotorDesiredEncoderPosition = Config.powerCellDistanceInEncoderCounts -
                            UpperIndexerMotor.getSelectedSensorPosition(0);
                    LowerMotorDesiredEncoderPosition = Config.powerCellDistanceInEncoderCounts
                            - LowerIndexerMotor.getSelectedSensorPosition(0);
                    UpperIndexerMotor.set(ControlMode.MotionMagic, UpperMotorDesiredEncoderPosition);
                    LowerIndexerMotor.set(ControlMode.MotionMagic, LowerMotorDesiredEncoderPosition);
                    incrementIndexerCounter();
                    //indexer is free for use by other commands.
                    isFinishedIndexing = true;
                    //indexer should be off.
                    indexerState = IndexerState.OFF;
                }
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motors to stop completely.
     *
     */

    //Make method that indexes balls properly while taking in multiple power cells at once.

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
            if(isSensorBlocked() && RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INTAKE) {
                indexIn();
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Updates the indexer's state machine.
     *
     * @param state - The state to which the state machine will be updated.
     *
     */
    public void updateIndexerState(IndexerState state) {
        indexerState = state;
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

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard.
     */
    public void outputToDashboard() {
        SmartDashboard.putNumber("Upper motor desired encoder position:", UpperMotorDesiredEncoderPosition);
        SmartDashboard.putNumber("Lower motor desired encoder position:", LowerMotorDesiredEncoderPosition);
        SmartDashboard.putNumber("Upper motor current encoder position:",
                UpperIndexerMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Lower motor current encoder position:",
                LowerIndexerMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Upper motor power:", UpperIndexerMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Lower motor power:", LowerIndexerMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Upper motor current:", Robot.pdp.getCurrent(Config.upperIndexerMotorPdpChannel));
        SmartDashboard.putNumber("Lower motor current:", Robot.pdp.getCurrent(Config.lowerIndexerMotorPdpChannel));
        SmartDashboard.putBoolean("Sensor blocked:", isSensorBlocked());
        SmartDashboard.putNumber("Current Power Cell count:", getCount());
    }
}