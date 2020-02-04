package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * @author Joshua Badzey
 *
 * The class for the shooter subsystem, which will launch the power cells to desired targets.
 * Will work in tandem with indexer to determine whether to shoot or not, and will work with
 * aimer and drive to determine what level of power to use to achieve an accurate shot.
 *
 */

public class ShooterSubsystem extends SubsystemBase {

    public enum shooterState{
        nothing,
        straightShot,
        backSpin,
        forwardSpin
    }

    private WPI_TalonFX topMotor;
    private WPI_TalonFX bottomMotor;

    public ShooterSubsystem() {
        topMotor = new WPI_TalonFX(Constants.SHOOTER_TOP_MOTOR_ID);
        bottomMotor = new WPI_TalonFX(Constants.SHOOTER_BOTTOM_MOTOR_ID);
        topMotor.configFactoryDefault();
        bottomMotor.configFactoryDefault();
    }

    //spins up the flywheel to a set speed, with a certain timeout value.
    public void SpinFlywheels(double speed, double timeOut) {

    }

    //immediately stops the flywheels.
    public void StopFlywheels() {
        topMotor.setNeutralMode(NeutralMode.Coast);
        topMotor.set(ControlMode.Velocity, 0);
        bottomMotor.setNeutralMode(NeutralMode.Coast);
        bottomMotor.set(ControlMode.Velocity, 0);

    }

    //differentiates the rate of spin for motors so that the power cell itself can spin predictably.
    public void SpinPowerCell(double topSpin, double bottomSpin) {
        topMotor.set(ControlMode.Velocity, topSpin);
        bottomMotor.set(ControlMode.Velocity, bottomSpin);
    }
}