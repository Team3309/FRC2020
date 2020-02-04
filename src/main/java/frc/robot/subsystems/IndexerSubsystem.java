package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * @author Joshua Badzey
 *
 * The class for the power cell indexer, which will move power cells within the robot and determine how many
 * there are. Will work with power cell intake and shooter to determine how many power cells there are at
 * any given moment.
 *
 */

public class IndexerSubsystem extends SubsystemBase {

    public enum indexerState {
        nothing,
        loading,
        ejecting
    }

    private WPI_TalonFX indexerMotor;

    public IndexerSubsystem() {
        indexerMotor = new WPI_TalonFX(Constants.INDEXER_MOTOR_ID);
        indexerMotor.configFactoryDefault();
    }
    //will detect whether indexer is full or not.
    public boolean IsFull() {return false;}
    //will detect whether indexer is empty or not
    public boolean IsEmpty() {return !IsFull();}
    //will load power cell into shooter or eject it into the intake; if (inOrOut), load; else if(!inOrOut), eject; else
    //stand still.
    public void Load() {}
    public void Eject() {}
    public void StopIndexer() {}
}
