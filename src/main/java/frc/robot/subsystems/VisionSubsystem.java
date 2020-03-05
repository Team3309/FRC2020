package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            limelight = new Limelight("limelight",
                    0, 0, 0);
            if (!Config.isLimelightOn) {
                limelight.setLed(Limelight.LEDMode.Off);
            }
        }
    }

    public double getAngleToTarget() {
        if (!Config.isVisionInstalled) {
            return 0;
        }
        return limelight.getTx();
    }

    public boolean hasTarget() {

        if (!Config.isVisionInstalled) {
            return true;
        }
        return limelight.hasTarget();
    }

    public double getDistanceToTarget() {
        //following the documentation at https://readthedocs.org/projects/limelight/downloads/pdf/latest/
        //in addition with a variant of a spherical coordiantes to obtain 3d cartesian coordinates.
        //requires us to use vision from a static position
        if (!Config.isVisionInstalled) {
            return 0;
        }
        double distanceToVisionTarget =
                Config.visionDistanceConstant * (Config.fieldVisionTargetHeight-Config.limelightMountingHeight) /
                Math.tan(Math.toRadians(limelight.getTy() + Config.limelightMountingAngle));


        //now we can convert the new values into a new distance

        return distanceToVisionTarget;
    }

    public void setIllumination (boolean illuminated) {
        if (illuminated)
            limelight.setLed(Limelight.LEDMode.On);
        else
            limelight.setLed(Limelight.LEDMode.Off);
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends limelight data to SmartDashboard.
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);

        double theta = Math.toRadians(getAngleToTarget()); // we rename these for sake of sanity with the math
        double phi = Math.toRadians(getHeightAngleToTarget());
        double depthToTarget = getDistanceToTarget() * Math.cos(phi) * Math.cos(theta);
        double lengthToTarget = getDistanceToTarget() * Math.cos(phi) * Math.sin(theta);
        double heightToTarget = getDistanceToTarget() * Math.sin(phi);

        //from here we can add value to depth height and length easily because we are relative to the back hole in its basis.
        double depthToThreePointGoal = depthToTarget + Config.fieldVisionDepthOfThreePointHoleFromVisionTarget;
        double lengthToThreePointGoal = lengthToTarget;
        double heightToThreePointGoal = heightToTarget + Config.fieldVisionHeightOfThreePointHoleFromVisionTarget;

        //
        double threePointHoleDistance = Math.sqrt(depthToThreePointGoal * depthToThreePointGoal + lengthToThreePointGoal * lengthToThreePointGoal + heightToThreePointGoal * heightToThreePointGoal);
        //WE'LL NEED THESE LATER HOLD ON TO THE FORMULAE FOR NOW
        double threePointHoleTx = Math.toDegrees(Math.asin(heightToThreePointGoal / threePointHoleDistance)); //aka adjusted phi, aka the angle we need to rotate by to be facing the 3 point goal
        double threePointHoleTy = Math.toDegrees(Math.atan2(lengthToThreePointGoal, depthToThreePointGoal)); //this and distance become the arm angle and power.

        SmartDashboard.putNumber("Three Point Hole Distance", threePointHoleDistance);
        SmartDashboard.putNumber("Tx (Adjusted)", threePointHoleTx);
        SmartDashboard.putNumber("Ty (Adjusted)", threePointHoleTy);
        SmartDashboard.putNumber("Vision Distance", getDistanceToTarget());

        SmartDashboard.putNumber("Distance to Alliance Wall (Depth)", depthToTarget);

    }

    public double getHeightAngleToTarget() {
        if (!Config.isVisionInstalled) {
            return 0;
        }
        return limelight.getTy();
    }
}
