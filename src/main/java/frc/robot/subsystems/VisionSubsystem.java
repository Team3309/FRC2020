package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.util.Limelight;

/**
 * @author Joshua Badzey
 *
 * The vision class for the vision subsystem, which will perform all Limelight related activities, such
 * as turning the LEDs on and off. Will work with drive to determine accurate drive paths and will also work
 * with aimer and shooter to determine optimal shot distance and power.
 *
 */

public class VisionSubsystem extends SubsystemBase {


    public Limelight limelight;

    public VisionSubsystem() {
        if (Config.isVisionInstalled) {
            limelight = new Limelight("Shooter Limelight",
                    0, 0, 0);
        }
    }

    public double getAngleToTarget() {
        return limelight.getTx();
    }

    public boolean hasTarget() {
        return limelight.hasTarget();
    }

    public double getDistanceToTarget() {
        //following the documentation at https://readthedocs.org/projects/limelight/downloads/pdf/latest/
        //in addition with a variant of a spherical coordiantes to obtain 3d cartesian coordinates.
        //requires us to use vision from a static position
        double distanceToVisionTarget =
                (Config.fieldVisionTargetHeight-Config.limelightMountingHeight) /
                Math.tan(limelight.getTy() + Config.limelightMountingAngle);


        //now we can convert the new values into a new distance

        return distanceToVisionTarget;
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends limelight data to SmartDashboard.
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }

    public double getHeightAngleToTarget() {
        return limelight.getTy();
    }
}
