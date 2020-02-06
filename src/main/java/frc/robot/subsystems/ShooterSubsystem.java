package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.RobotContainer;

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
    private RobotContainer robotContainer;


    public ShooterSubsystem() {
        /*topMotor = new WPI_TalonFX(Config.TopShooterMotorID);
        bottomMotor = new WPI_TalonFX(Config.BottomShooterMotorID);
        ConfigTalon(topMotor);
        ConfigTalon(bottomMotor);*/
    }

    public void ConfigTalon(WPI_TalonFX talon) {

        talon.configFactoryDefault();;

        talon.configClosedloopRamp(Config.ShooterClosedLoopRampRate);
        talon.configOpenloopRamp(Config.ShooterOpenLoopRampRate, 10);

        talon.config_kP(0, Config.ShooterVelocityP, 10);
        talon.config_kI(0, Config.ShooterVelocityI, 10);
        talon.config_IntegralZone(0, Config.ShooterVelocityIntegralZone, 10);
        talon.config_kD(0, Config.ShooterVelocityD, 10);
        talon.config_kF(0, Config.ShooterVelocityF, 10);

        talon.setNeutralMode(NeutralMode.Coast);
        talon.setInverted(false);
        talon.setSensorPhase(false);
    }

    //spins up the flywheel to a set speed, with a certain timeout value.
    public void SpinUpFlywheels(double speed) {


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

    public double GetTopMotorVelocity() {
        return topMotor.getSelectedSensorVelocity();
    }

    public double GetBottomMotorVelocity() {
        return -bottomMotor.getSelectedSensorVelocity();
    }
}
