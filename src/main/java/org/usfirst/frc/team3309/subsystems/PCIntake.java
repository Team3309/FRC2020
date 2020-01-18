package org.usfirst.frc.team3309.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc.team3309.Constants;

/**
 * @author Joshua Badzey
 *
 * The class for the power cell intake, which will intake or expel power cells. Will work with shooter and
 * indexer to move power cells around within the robot.
 *
 */
public class PCIntake extends SubsystemBase {
    private WPI_TalonFX intakeMotor;

    public PCIntake() {
        intakeMotor = new WPI_TalonFX(Constants.INTAKE_MOTOR_ID);
    }
    public void intake() {}
    public void outtake() {}
}
