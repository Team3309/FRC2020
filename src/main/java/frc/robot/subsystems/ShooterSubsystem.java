package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;
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

    private Timer controlTimer = new Timer();

    public ShooterSubsystem() {
        topMotor = new WPI_TalonFX(Constants.kTopShooterMotorPdpChannel);
        bottomMotor = new WPI_TalonFX(Constants.kBottomShooterMotorPdpChannel);
        topMotor.configFactoryDefault();
        bottomMotor.configFactoryDefault();
    }

    //spins up the flywheel to a set speed, with a certain timeout value.
    public void SpinFlywheels(double speed, double timeOut) {
        controlTimer.start();

        while (controlTimer.get() < timeOut) {
            topMotor.set(speed);
            bottomMotor.set(speed);
        }

        if (controlTimer.get() >= timeOut) {
            topMotor.set(ControlMode.PercentOutput, 0);
            topMotor.set(ControlMode.PercentOutput, 0);
            controlTimer.stop();
            controlTimer.reset();
        }
    }

    //immediately stops the flywheels.
    public void StopFlywheels() {
        topMotor.set(ControlMode.PercentOutput, 0);
        bottomMotor.set(ControlMode.PercentOutput, 0);
    }

    //differentiates the rate of spin for motors so that the power cell itself can spin predictably.
    public void SpinPowerCell(double topSpin, double bottomSpin) {
        topMotor.set(ControlMode.Velocity, topSpin);
        bottomMotor.set(ControlMode.Velocity, bottomSpin);
    }

    //above method, but with a timeout.
    public void SpinPowerCell(double topSpin, double bottomSpin, double timeOut) {

        controlTimer.start();
        while (controlTimer.get() < timeOut) {
            topMotor.set(topSpin);
            bottomMotor.set(bottomSpin);
        }

        if (controlTimer.get() >= timeOut) {
            topMotor.set(ControlMode.PercentOutput, 0);
            bottomMotor.set(ControlMode.PercentOutput, 0);
        }
    }
}
