package org.usfirst.frc.team3309.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc.team3309.Constants;

/**
 * @author Joshua Badzey
 *
 * The class for the balancer subsystem, which will adjust the robot's position on the generator switch rung.
 * Will work with the climber to determine whether the robot is physically on the rung, and will use IMU data
 * to determine how far off from horizontal the robot is based on tilt.
 *
 */

public class Balancer extends SubsystemBase {

    private WPI_TalonSRX balancerMotor;
    public Balancer() {
        balancerMotor = new WPI_TalonSRX(Constants.BALANCER_MOTOR_ID);
    }
    //move the robot on the rung; its left is negative, and its right is positive.
    public void changePosition(double distance) {

    }
}
