package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;


/**---------------------------------------------------------------------------------------------------------------------
 * @author Joshua Badzey
 *
 * The class for the power cell indexer, which will move power cells within the robot and determine how many
 * there are. It involves both the internal belt of the power cell containment area and the arm of the shooter.
 * Will work with power cell intake and shooter to determine how many power cells there are atany given moment.
 *
 */

public class IndexerSubsystem extends SubsystemBase {

    public enum IndexerState {
        OFF,
        INDEXING,
        EJECTING
    }

    private IndexerState indexerState = IndexerState.OFF;
    private WPI_TalonSRX UpperIndexerMotor;
    private WPI_TalonSRX LowerIndexerMotor;
    private int UpperMotorDesiredEncoderPosition;
    private int LowerMotorDesiredEncoderPosition;
    private DigitalInput PowerCellSensor;
    private int PowerCells;

    public IndexerSubsystem() {
        if (Config.isIndexerInstalled) {
            UpperIndexerMotor = new WPI_TalonSRX(Config.PrimaryIndexerMotorID);
            UpperMotorDesiredEncoderPosition = UpperIndexerMotor.getSelectedSensorPosition(0);
            LowerIndexerMotor = new WPI_TalonSRX(Config.SecondaryIndexerMotorID);
            LowerMotorDesiredEncoderPosition = LowerIndexerMotor.getSelectedSensorPosition(0);
            UpperIndexerMotor.config_kP(0, Config.IndexerP);
            UpperIndexerMotor.config_kI(0, Config.IndexerI);
            UpperIndexerMotor.config_IntegralZone(0, Config.IndexerIntegralZone);
            UpperIndexerMotor.config_kD(0, Config.IndexerD);
            UpperIndexerMotor.config_kF(0, Config.IndexerF);
            LowerIndexerMotor.config_kP(0, Config.IndexerP);
            LowerIndexerMotor.config_kI(0, Config.IndexerI);
            LowerIndexerMotor.config_IntegralZone(0, Config.IndexerIntegralZone);
            LowerIndexerMotor.config_kD(0, Config.IndexerD);
            LowerIndexerMotor.config_kF(0, Config.IndexerF);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to roll the belt forward. Zeroes the relative encoder position when finished.
     *
     */
    public void indexOut() {
        if (Config.isIndexerInstalled) {
            if (indexerState == IndexerState.EJECTING) {
                UpperMotorDesiredEncoderPosition = UpperIndexerMotor.getSelectedSensorPosition(0)
                        - Config.StandardIndexerMotionInEncoderCounts;
                LowerMotorDesiredEncoderPosition = LowerIndexerMotor.getSelectedSensorPosition(0)
                        - Config.StandardIndexerMotionInEncoderCounts;
                UpperIndexerMotor.set(ControlMode.MotionMagic, UpperMotorDesiredEncoderPosition);
                LowerIndexerMotor.set(ControlMode.MotionMagic, LowerMotorDesiredEncoderPosition);
                if (PowerCells > 0) {
                    decrementIndexerCounter();
                }
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to roll the belt backward. Zeroes the relative encoder position when finished.
     *
     */
    private void indexIn() {
        if (Config.isIndexerInstalled) {
            if (isInPosition()) {
                if (indexerState == IndexerState.INDEXING && PowerCells < 5) {
                    UpperMotorDesiredEncoderPosition = Config.StandardIndexerMotionInEncoderCounts -
                            UpperIndexerMotor.getSelectedSensorPosition(0);
                    LowerMotorDesiredEncoderPosition = Config.StandardIndexerMotionInEncoderCounts
                            - LowerIndexerMotor.getSelectedSensorPosition(0);
                    UpperIndexerMotor.set(ControlMode.MotionMagic, UpperMotorDesiredEncoderPosition);
                    LowerIndexerMotor.set(ControlMode.MotionMagic, LowerMotorDesiredEncoderPosition);
                    incrementIndexerCounter();

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
    public boolean sensorBlocked() {
        return !PowerCellSensor.get();
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Increments the counter for the number of power cells currently in the indexer.
     *
     */
    public void incrementIndexerCounter() {
        PowerCells++;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Decrements the counter for the number of power cells currently in the indexer.
     *
     */
    public void decrementIndexerCounter() {
        PowerCells--;
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
     * Gets the current velocity of the upper motor.
     *
     * @return The velocity of the upper motor in encoder counts per 100 milliseconds.
     *
     */
    public double getPrimaryMotorVelocity() {
        if (Config.isIndexerInstalled) {
            return UpperIndexerMotor.getSelectedSensorVelocity();
        } else {
            return 0;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Gets the current velocity of the lower motor.
     *
     * @return The velocity of the lower motor in encoder counts per 100 milliseconds.
     *
     */
    public double getSecondaryMotorVelocity() {
        if (Config.isIndexerInstalled) {
            return LowerIndexerMotor.getSelectedSensorVelocity();
        } else {
            return 0;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Checks the state of the beam-break sensor, and moves the indexer based on that state.
     *
     */
    public void manageSensorState() {
        if (Config.isIndexerInstalled) {
            if(sensorBlocked() && indexerState == IndexerState.INDEXING) {
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
                < Config.IndexerMaximumEncoderPositionRange)) && (Math.abs(LowerMotorDesiredEncoderPosition
                - LowerIndexerMotor.getSelectedSensorPosition()) < Config.IndexerMaximumEncoderPositionRange);
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard.
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }
}
