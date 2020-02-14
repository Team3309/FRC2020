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

    private enum IndexerState {
        OFF,
        INDEXING,
        EJECTING
    }

    private IndexerState indexerState = IndexerState.OFF;
    private WPI_TalonSRX PrimaryIndexerMotor;
    private WPI_TalonSRX SecondaryIndexerMotor;
    private double IndexerPosition = 0;
    private DigitalInput Sensor;
    private int PowerCells;


    public IndexerSubsystem() {
        if (Config.isIndexerInstalled) {
            PrimaryIndexerMotor = new WPI_TalonSRX(Config.PrimaryIndexerMotorID);
            SecondaryIndexerMotor = new WPI_TalonSRX(Config.SecondaryIndexerMotorID);
            PrimaryIndexerMotor.config_kP(1, Config.IndexerP);
            PrimaryIndexerMotor.config_kI(1, Config.IndexerI);
            PrimaryIndexerMotor.config_IntegralZone(1, Config.IndexerIntegralZone);
            PrimaryIndexerMotor.config_kD(1, Config.IndexerD);
            PrimaryIndexerMotor.config_kF(1, Config.IndexerF);
            SecondaryIndexerMotor.config_kP(2, Config.IndexerP);
            SecondaryIndexerMotor.config_kI(2, Config.IndexerI);
            SecondaryIndexerMotor.config_IntegralZone(2, Config.IndexerIntegralZone);
            SecondaryIndexerMotor.config_kD(2, Config.IndexerD);
            SecondaryIndexerMotor.config_kF(2, Config.IndexerF);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to roll the belt forward. Zeroes the relative encoder position when finished.
     *
     */
    public void indexOut() {
        if (Config.isIndexerInstalled) {
            if (!sensorBlocked()) {
                indexerState = IndexerState.EJECTING;
                if (indexerState == IndexerState.EJECTING) {
                    PrimaryIndexerMotor.set(ControlMode.Position, IndexerPosition
                            - Config.StandardIndexerMotionInEncoderCounts);
                    SecondaryIndexerMotor.set(ControlMode.Position,
                            Config.StandardIndexerMotionInEncoderCounts - IndexerPosition);
                    IndexerPosition = 0;
                    if (PowerCells > 0) {
                        decrementIndexerCounter();
                    }
                }
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to roll the belt backward. Zeroes the relative encoder position when finished.
     *
     */
    public void indexIn() {
        if (Config.isIndexerInstalled) {
            if (sensorBlocked()) {
                indexerState = IndexerState.INDEXING;
                if (indexerState == IndexerState.INDEXING && PowerCells < 5) {
                    PrimaryIndexerMotor.set(ControlMode.Position,
                            Config.StandardIndexerMotionInEncoderCounts - IndexerPosition);
                    SecondaryIndexerMotor.set(ControlMode.Position,
                            IndexerPosition - Config.StandardIndexerMotionInEncoderCounts);
                    IndexerPosition = 0;
                    if (PowerCells < 5) {
                        incrementIndexerCounter();
                    }
                }
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motors to stop completely.
     *
     */
    public void stopIndexer() {
        if (Config.isIndexerInstalled) {
            indexerState = IndexerState.OFF;
            if (indexerState == IndexerState.OFF) {
                PrimaryIndexerMotor.set(ControlMode.PercentOutput, 0.0);
                SecondaryIndexerMotor.set(ControlMode.PercentOutput, 0.0);
                IndexerPosition = 0;
            }
        }
    }

    //Make method that indexes balls properly while taking in multiple power cells at once.

    public boolean sensorBlocked() {
        return !Sensor.get();
    }

    public void incrementIndexerCounter() {
        PowerCells++;
    }

    public void decrementIndexerCounter() {
        PowerCells--;
    }

    public int getCount() {
        return PowerCells;
    }

    public double getPrimaryMotorVelocity() {
        return PrimaryIndexerMotor.getSelectedSensorVelocity();
    }
    public double getSecondaryMotorVelocity() {
        return SecondaryIndexerMotor.getSelectedSensorVelocity();
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard.
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }
}
