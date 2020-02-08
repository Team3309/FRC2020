package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.RobotContainer;


/**---------------------------------------------------------------------------------------------------------------------
 * @author Joshua Badzey
 *
 * The class for the power cell indexer, which will move power cells within the robot and determine how many
 * there are. It involves both the internal belt of the power cell containment area and the arm of the shooter.
 * Will work with power cell intake and shooter to determine how many power cells there are atany given moment.
 *
 ---------------------------------------------------------------------------------------------------------------------*/

public class IndexerSubsystem extends SubsystemBase {

    private WPI_TalonFX indexerMotor;
    private RobotContainer robotContainer;

    public IndexerSubsystem() {
        //indexerMotor = new WPI_TalonFX(Config.IndexerMotorID);
        //indexerMotor.configFactoryDefault();
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Finds out whether power cell containment area is full based on the change of belt motor electrical behavior,
     * i.e. if the belt motor is set to a certain velocity and the belt motor has to draw more power to keep it at
     * speed, the robot can know that something has entered the power cell containment area.
     *
     * Joshua: We could set up a counter for tracking how many power cells are in the indexer at a time.
     *
     * ^^^
     * |||
     * |||
     * UP FOR REVIEW AND DISCUSSION.
     *
     * @return Whether or not the indexer is full.
     *
     -----------------------------------------------------------------------------------------------------------------*/
    public boolean IsFull() {return false;}

    /**-----------------------------------------------------------------------------------------------------------------
     * Finds out whether the power cell containment area is completely empty of items based on similar electrical
     * behavior monitoring to IsFull.
     *
     * @return Whether or not the indexer is completely empty.
     *
     -----------------------------------------------------------------------------------------------------------------*/
    public boolean IsEmpty() {return false;}

    /**-----------------------------------------------------------------------------------------------------------------
     * Loads a power cell forward into the shooter. The shooter should ideally have spun up its motors to desired
     * flywheel speeds.
     *
     -----------------------------------------------------------------------------------------------------------------*/
    public void Load() {
        indexerMotor.set(Config.IndexerStandardVelocity);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Ejects a power cell from the indexer using the shooter. Will not need the same level of speed for the flywheels
     * as the shooter.
     *
     -----------------------------------------------------------------------------------------------------------------*/
    public void Eject() {}

    //We need to discuss what this method does. @MarkG
    public void StopIndexer() {}
}
