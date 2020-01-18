package org.usfirst.frc.team3309.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc.team3309.Constants;

/**
 * @author Joshua Badzey
 *
 * The class for the shooter subsystem, which will launch the power cells to desired targets.
 * Will work in tandem with indexer to determine whether to shoot or not, and will work with
 * aimer and drive to determine what level of power to use to achieve an accurate shot.
 *
 */

public class Shooter extends SubsystemBase {
    private WPI_TalonFX topMotor;
    private WPI_TalonFX bottomMotor;
    public Shooter() {
        topMotor = new WPI_TalonFX(Constants.SHOOTER_TOP_MOTOR_ID);
        bottomMotor = new WPI_TalonFX(Constants.SHOOTER_BOTTOM_MOTOR_ID);
    }

    //spins up the flywheel to a set speed, with a certain timeout value.
    public void spinFlywheel(double speed, double timeOut) {}
    //stops the flywheel cold.
    public void stopFlywheel() {}
    //differentiates the rate of spin for motors so that the power cell itself can spin predictably.
    public void spinPowerCell(double topSpin, double bottomSpin) {
        topMotor.set(ControlMode.Velocity, topSpin);
        bottomMotor.set(ControlMode.Velocity, bottomSpin);
    }
}
