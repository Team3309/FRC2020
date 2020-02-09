package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class ArmSubsystem extends SubsystemBase {

    public int desiredPosition;
    public int MAXIMUM_ENCODER_DISTANCE_FOR_POSITION = 3; //maximum encoder distance to be properly in a position

    public boolean isInPosition() {
        return !Config.isArmInstalled || Math.abs(armMotor.getSelectedSensorPosition(0) - desiredPosition) < MAXIMUM_ENCODER_DISTANCE_FOR_POSITION;
    }

    public enum ArmPosition {
        max(0),
        longRange(1),
        midRange(2),
        closeRange(3),
        trench(4),
        min(5),
        intermediate(6);

        int value;

        ArmPosition(int value) {
            this.value = value;
        }
    }

    private WPI_TalonFX armMotor;


    public ArmSubsystem() {
        if (Config.isArmInstalled) {
            armMotor = new WPI_TalonFX(Config.ArmMotorId);
        }
    }

    /**
     * ---------------------------------------------------------------------------------------------------------------\
     * Moves arm based on a certain number of encoder counts.
     *
     * @param position - By how many encoder counts the arm should move.
     *                 <p>
     *                 \----------------------------------------------------------------------------------------------------------------
     */
    public void MoveArmManually(double position) {
        if (Config.isArmInstalled) {
            armMotor.set(ControlMode.Position, position);
        }
    }

    //just a stub method for now
    public void zeroEncoder() {

    }

    /**
     * ---------------------------------------------------------------------------------------------------------------\
     * Moves arm to a preset Arm position. Most of this is done for us properly with the PIDs and MotionMagic.
     *
     * @param position - The Arm position to which the arm will move.
     *
     */
    public void MoveToPosition(ArmPosition position) {
        if (Config.isArmInstalled) {
            armMotor.set(ControlMode.Position, position.value);
        }
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }
}
