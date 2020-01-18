package org.usfirst.frc.team3309.subsystems;

<<<<<<< HEAD
import org.usfirst.frc.team3309.util.Limelight;
=======
import edu.wpi.first.wpilibj2.command.SubsystemBase;
>>>>>>> 66fabc297ab8ee6e2af8c8aa432702c24c03985c

/**
 * @author Joshua Badzey
 *
 * The vision class for the vision subsystem, which will perform all Limelight related activities, such
 * as turning the LEDs on and off. Will work with drive to determine accurate drive paths and will also work
 * with aimer and shooter to determine optimal shot distance and power.
 *
 */

<<<<<<< HEAD
public class Vision {

    public Limelight limelight = new Limelight("Shooter Limelight", 0, 0 ,0);

=======
public class Vision extends SubsystemBase {
>>>>>>> 66fabc297ab8ee6e2af8c8aa432702c24c03985c
    public Vision() {}
}
