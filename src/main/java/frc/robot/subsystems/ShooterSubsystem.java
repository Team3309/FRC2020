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


    private WPI_TalonFX topMotor;
    private WPI_TalonFX bottomMotor;


    public ShooterSubsystem() {
        topMotor = new WPI_TalonFX(Constants.SHOOTER_TOP_MOTOR_ID);
        bottomMotor = new WPI_TalonFX(Constants.SHOOTER_BOTTOM_MOTOR_ID);
        ConfigTalon(topMotor);
        ConfigTalon(bottomMotor);
    }

    public void ConfigTalon(WPI_TalonFX talon) {

        talon.configFactoryDefault();;

        talon.configClosedloopRamp(Constants.ShooterClosedLoopRampRate);
        talon.configOpenloopRamp(Constants.ShooterOpenLoopRampRate, 10);

        talon.config_kP(0, Constants.ShooterVelocityP, 10);
        talon.config_kI(0, Constants.ShooterVelocityI, 10);
        talon.config_IntegralZone(0, Constants.ShooterVelocityIntegralZone, 10);
        talon.config_kD(0, Constants.ShooterVelocityD, 10);
        talon.config_kF(0, Constants.ShooterVelocityF, 10);

        talon.setNeutralMode(NeutralMode.Coast);
        talon.setInverted(false);
        talon.setSensorPhase(false);
    }

    //spins up the flywheel to a set speed, with a certain timeout value.
    public void SpinUpFlywheels(double speed, double timeOut) {


    }

    //immediately stops the flywheels.
    public void StopFlywheels() {
        topMotor.set(ControlMode.PercentOutput, 0);
        bottomMotor.set(ControlMode.PercentOutput, 0);
    }

    //differentiates the rate of spin for motors so that the power cell itself can spin predictably.
    public void SpinPowerCell(double topSpeed, double bottomSpeed) {
        topMotor.set(ControlMode.Velocity, topSpeed);
        bottomMotor.set(ControlMode.Velocity, bottomSpeed);
    }

    //above method, but with a timeout.
    public void SpinPowerCell(double topSpeed, double bottomSpeed, double timeOut) {

    }

    public double GetTopMotorVelocity() {
        return topMotor.getSelectedSensorVelocity();
    }

    public double GetBottomMotorVelocity() {
        return -bottomMotor.getSelectedSensorVelocity();
    }
}
