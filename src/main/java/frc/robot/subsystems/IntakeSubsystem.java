package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.RobotContainer;

 /**-------------------------------------------------------------------------------------------------------------------\
 * @author Joshua Badzey
 *
 * The class for the power cell intake, which will intake or expel power cells. Will work with shooter and
 * indexer to move power cells around within the robot.
 *
 \--------------------------------------------------------------------------------------------------------------------*/
public class IntakeSubsystem extends SubsystemBase {

    private WPI_TalonFX intakeMotor;
    private Solenoid solenoid;

    public IntakeSubsystem() {
        if (Config.isIntakeInstalled) {
            intakeMotor = new WPI_TalonFX(Config.IntakeMotorID);
            intakeMotor.configFactoryDefault();
            solenoid = new Solenoid(Config.IntakeSoleoidChannel);
        }
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Activates the motor to spin at @param speed.
     *
     * @param speed - the speed at which the motors should turn the intake wheels.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void Spin(double speed) {
        intakeMotor.set(ControlMode.Velocity, speed);
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Spins the intake wheels for intaking a power cell.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void Intake() { Spin(Config.IntakeInwardPower); }

     /**---------------------------------------------------------------------------------------------------------------\
     * Spins the intake wheels for outtaking a power cell.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void Outtake() { Spin(-Config.IntakeInwardPower); }

     /**---------------------------------------------------------------------------------------------------------------\
     * Activates intake piston to extend the intake forward.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void Extend() {
        solenoid.set(true);
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Deactivates the intake piston to retract the intake back.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void Retract() {
        solenoid.set(false);
    }
}
