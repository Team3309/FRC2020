package org.usfirst.frc.team3309.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * @author Joshua Badzey
 *
 * The class for the aimer subsystem, which will aim the shooter at the correct target.
 * Will work in tandem with shooter and drive to determine best angle for power cell shots.
 *
 */

public class Aimer extends SubsystemBase {
    public Aimer() {}
    //changes where the robot is aiming.
    public void changeAim(double degrees) {}
    //changes the robot's aim at a set speed
    public void changeAim(double degrees, double speed) {}
}
