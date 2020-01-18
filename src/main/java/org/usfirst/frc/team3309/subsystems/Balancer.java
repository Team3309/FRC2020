package org.usfirst.frc.team3309.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * @author Joshua Badzey
 *
 * The class for the balancer subsystem, which will adjust the robot's position on the generator switch rung.
 * Will work with the climber to determine whether the robot is physically on the rung, and will use IMU data
 * to determine how far off from horizontal the robot is based on tilt.
 *
 */

public class Balancer extends SubsystemBase {

    public Balancer() {}
}
