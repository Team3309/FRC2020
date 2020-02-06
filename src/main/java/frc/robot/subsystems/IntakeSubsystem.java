package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.RobotContainer;

/**
 * @author Joshua Badzey
 *
 * The class for the power cell intake, which will intake or expel power cells. Will work with shooter and
 * indexer to move power cells around within the robot.
 *
 */
public class IntakeSubsystem extends SubsystemBase {

    public RobotContainer robotContainer;
    public WPI_TalonFX intakeMotor;
    public Solenoid solenoid;

    public IntakeSubsystem(RobotContainer container) {
        intakeMotor = new WPI_TalonFX(Config.IntakeMotorID);
        intakeMotor.configFactoryDefault();
        robotContainer = container;
        solenoid = new Solenoid(Config.IntakeSoleoidChannel);
    }

    public void Spin(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void Extend() {
        solenoid.set(true);
    }

    public void Retract() {
        solenoid.set(false);
    }
}
