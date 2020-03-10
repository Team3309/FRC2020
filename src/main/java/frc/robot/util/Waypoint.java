package frc.robot.util;

import frc.robot.Config;

public class Waypoint {

    public double downFieldInches; //how far the waypoint is from the driver station
    public double xFieldInches; //lateral position of the waypoint
    public double turnRadiusInches; //centered on the vertices of the straight-line path, not the guide circles
    public boolean reverse;  // robot backs into waypoint
    public Double poseDegrees;  // positive = counter-clockwise

    public double linCreepSpeedInInchesPerSec = Config.linCreepSpeedInInchesPerSec;
    public double angCreepSpeedInDegsPerSec = Config.angCreepSpeedInDegsPerSec;
    public double maxLinearSpeedInInchesPerSec = Config.maxLinearSpeedInInchesPerSec;
    public double maxAngularSpeedInDegsPerSec = Config.maxAngularSpeedInDegsPerSec;
    public double linAccelerationInInchesPerSec2 = Config.linAccelerationInInchesPerSec2;
    public double linDecelerationInInchesPerSec2 = Config.linDecelerationInInchesPerSec2;
    public double angAccelerationInDegsPerSec2 = Config.angAccelerationInDegsPerSec2;
    public double angDecelerationInDegsPerSec2 = Config.angDecelerationInDegsPerSec2;
    public double linearToleranceInInches = Config.linearToleranceInInches;

    public Waypoint(double downFieldInches,
                    double xFieldInches,
                    double turnRadiusInches,
                    boolean reverse) {
        this.downFieldInches = downFieldInches;
        this.xFieldInches = xFieldInches;
        this.turnRadiusInches = turnRadiusInches;
        this.reverse = reverse;
    }

    public Waypoint(double downFieldInches,
                    double xFieldInches,
                    double turnRadiusInches,
                    boolean reverse,
                    double poseDegrees) {
        this.downFieldInches = downFieldInches;
        this.xFieldInches = xFieldInches;
        this.turnRadiusInches = turnRadiusInches;
        this.reverse = reverse;
        this.poseDegrees = poseDegrees;
    }
}
