package org.usfirst.frc.team3309.subsystems;

import org.usfirst.frc.team3309.util.Limelight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * @author Joshua Badzey
 *
 * The vision class for the vision subsystem, which will perform all Limelight related activities, such
 * as turning the LEDs on and off. Will work with drive to determine accurate drive paths and will also work
 * with aimer and shooter to determine optimal shot distance and power.
 *
 */

public class VisionSubsystem extends SubsystemBase {

    public enum visionState {
        nothing,
        ledOn,
        ledOff,
        calculating
    }

    public Limelight limelight = new Limelight("Shooter Limelight", 0, 0, 0);

    public VisionSubsystem() {

    }
}
