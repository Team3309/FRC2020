package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

 /** -------------------------------------------------------------------------------------------------------------------
 * The class for the shooter subsystem, which will launch the power cells to desired targets.
 * Will work in tandem with indexer to determine whether to shoot or not, and will work with
 * aimer and drive to determine what level of power to use to achieve an accurate shot.
 */
public class ShooterSubsystem extends SubsystemBase {

     /**
      * Flywheel
      */
    private double flywheelSpeedTop;
    private double flywheelSpeedBottom;

     public boolean isFlywheelToSpeed() {
         return Math.abs(GetTopMotorVelocity() - flywheelSpeedTop) < 0.01 &&
                 Math.abs(GetTopMotorVelocity() - flywheelSpeedTop) < 0.01;
     }

    private WPI_TalonFX topMotor;
    private WPI_TalonFX bottomMotor;

    public ShooterSubsystem() {
        if (Config.isShooterInstalled) {
            topMotor = new WPI_TalonFX(Config.TopShooterMotorID);
            bottomMotor = new WPI_TalonFX(Config.BottomShooterMotorID);
            ConfigTalon(topMotor);
            ConfigTalon(bottomMotor);
        }
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Configures shooter motors to shooter constants, including PID values, ramp rates and motor settings.
     *
     * @param talon - the talon to be configured.
     */
    public void ConfigTalon(WPI_TalonFX talon) {

        talon.configFactoryDefault();;

        talon.configClosedloopRamp(Config.shooterClosedLoopRampRate);
        talon.configOpenloopRamp(Config.shooterOpenLoopRampRate, 10);

        talon.config_kP(0, Config.shooterVelocityP, 10);
        talon.config_kI(0, Config.shooterVelocityI, 10);
        talon.config_IntegralZone(0, Config.shooterVelocityIntegralZone, 10);
        talon.config_kD(0, Config.shooterVelocityD, 10);
        talon.config_kF(0, Config.shooterVelocityF, 10);

        talon.setNeutralMode(NeutralMode.Coast);
        talon.setInverted(false);
        talon.setSensorPhase(false);
    }

     /** ---------------------------------------------------------------------------------------------------------------
     * Spins up the flywheels in preparation for firing.
     */
    public void SpinUpFlywheels() {
        SetPowerRaw(flywheelSpeedTop, flywheelSpeedBottom);
    }

     /** ---------------------------------------------------------------------------------------------------------------
     * Stops the flywheels by having them slow down to a complete stop.
     */
    public void StopFlywheels() {
        if (Config.isShooterInstalled) {
            topMotor.set(ControlMode.PercentOutput, 0);
            bottomMotor.set(ControlMode.PercentOutput, 0);
        }
    }

     /** ---------------------------------------------------------------------------------------------------------------
     * Spins up the flywheels at different velocities so that backspin or forward spin can be achieved in addition to a
     * straight shot.
     *
     * @param topSpeed - the desired speed of the top motor.
     * @param bottomSpeed - the desired speed of the bottom motor.
     */
    public void SetPowerRaw(double topSpeed, double bottomSpeed) {
        if (Config.isShooterInstalled) {
            topMotor.set(ControlMode.Velocity, topSpeed);
            bottomMotor.set(ControlMode.Velocity, bottomSpeed);
        }
    }

     /** ---------------------------------------------------------------------------------------------------------------
      * Stores motor speed for both motors equally
      *
      * @param speedTop - stores speed for top motor for later engagement
      * @param speedBottom - stores speed for bottom motor for later engagement
      */
     public void setDesiredSpeed(double speedTop, double speedBottom) {
         flywheelSpeedTop = speedTop;
         flywheelSpeedBottom = speedBottom;
     }

     /** ---------------------------------------------------------------------------------------------------------------
     * Returns the top motor's current speed.
     *
     * @return The top motor's current speed.
     */
    public double GetTopMotorVelocity() {
        if (Config.isShooterInstalled) {
            return topMotor.getSelectedSensorVelocity();
        }
        return 0;
    }

     /** ---------------------------------------------------------------------------------------------------------------
     * Returns the bottom motor's current speed.
     *
     * @return The bottom motor's current speed.
     */
    public double GetBottomMotorVelocity() {
        if (Config.isShooterInstalled) {
            return -bottomMotor.getSelectedSensorVelocity();
        }
        return 0;
    }

     /** ---------------------------------------------------------------------------------------------------------------
      * Sends motor data to SmartDashboard
      */
     public void outputToDashboard() {
         //SmartDashboard.putNumber("Key", value);
     }
}
