package org.usfirst.frc.team3309.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * @author Joshua Badzey
 *
 * The climber class for the climber subsystem, which will grab the shield generator switch and pull the robot up.
 * Will work with drive to determine whether it should begin climbing.
 *
 */

public class Climber extends SubsystemBase {
    public Climber() {}
    //will lift up the climber mechanism to grab on to the rung.
    public void liftClimber() {}
    //will change the extension of the climber mechanism; negative is contraction, positive is extension.
    public void changeHeight(double height) {}
}
