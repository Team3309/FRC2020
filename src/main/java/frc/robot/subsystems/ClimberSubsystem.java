package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/**
 * @author Joshua Badzey
 *
 * The climber class for the climber subsystem, which will grab the shield generator switch and pull the robot up.
 * Will work with drive to determine whether it should begin climbing.
 *
 */

public class ClimberSubsystem extends SubsystemBase {

    private WPI_TalonSRX climberMotor;

    public ClimberSubsystem() {
        climberMotor = new WPI_TalonSRX(Constants.BALANCER_MOTOR_ID);
    }

    //will lift up the climber mechanism to grab on to the rung.
    public void LiftClimber() {}
    //will change the extension of the climber mechanism; negative is contraction, positive is extension.
    public void ChangeHeight(double height) {}
}
