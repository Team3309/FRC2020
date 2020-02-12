package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
        OFF(0),
        INDEXING(1),
        EJECTING(2);


        private int value;

        IndexerState(int value) {this.value = value;}
    }

    private IndexerState indexerState = IndexerState.OFF;
    private WPI_TalonSRX PrimaryIndexerMotor;
    private WPI_TalonSRX SecondaryIndexerMotor;
    private double IndexerPosition = 0;


    public IndexerSubsystem() {
        if (Config.isIndexerInstalled) {
            PrimaryIndexerMotor = new WPI_TalonSRX(Config.IndexerMotorID);
            PrimaryIndexerMotor.config_kP(1, Config.IndexerP);
            PrimaryIndexerMotor.config_kI(1, Config.IndexerI);
            PrimaryIndexerMotor.config_IntegralZone(1, Config.IndexerIntegralZone);
            PrimaryIndexerMotor.config_kD(1, Config.IndexerD);
            PrimaryIndexerMotor.config_kF(1, Config.IndexerF);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to roll the belt forward. Zeroes the relative encoder position when finished.
     *
     */
    public void indexerOut() {
        if (Config.isIndexerInstalled) {
            indexerState = IndexerState.EJECTING;
            if (indexerState.value == 2) {
                PrimaryIndexerMotor.set(ControlMode.Position, IndexerPosition-Config.StandardIndexerMotionInEncoderCounts);
                IndexerPosition = 0;
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the indexer motor to roll the belt backward. Zeroes the relative encoder position when finished.
     *
     */
    public void indexerIn() {
        if (Config.isIndexerInstalled) {
            indexerState = IndexerState.INDEXING;
            if (indexerState.value == 1) {
                PrimaryIndexerMotor.set(ControlMode.Position, Config.StandardIndexerMotionInEncoderCounts-IndexerPosition);
                IndexerPosition = 0;
            }
        }
    }

    /***/
    public void stopIndexer() {
        if (Config.isIndexerInstalled) {
            indexerState = IndexerState.OFF;
            if (indexerState.value == 0) {
                PrimaryIndexerMotor.set(ControlMode.PercentOutput, 0.0);
                IndexerPosition = 0;
            }
        }
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard.
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }
}
