package frc.robot.util;

public class Waypoint {

    public boolean finalHeading;
    public double downFieldInches; //how far the waypoint is from the driver station
    public double xFieldInches; //lateral position of the waypoint
    public double turnRadiusInches; //centered on the vertices of the straight-line path, not the guide circles
    public boolean reverse;  // robot backs into waypoint

    public double linCreepSpeed = 5; //Inches per second
    public double angCreepSpeedInDegsPerSec = 2;
    public double maxLinearSpeed = 40; //Inches per 100 milliseconds
    public double maxAngularSpeedInDegsPerSec = 50;
    public double linAccelerationInInchesPerSec2 = 80; //Inches per 100 milliseconds^2
    public double linDecelerationInInchesPerSec2 = 160; //Also in inches per 100 milliseconds^2
    public double angAccelerationInDegsPerSec2 = 100;
    public double angDecelerationInDegsPerSec2 = 100;
    public double linToleranceInInches;

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
                    double maxLinearSpeed,
                    double maxAngularSpeedInDegsPerSec,
                    double linCreepSpeed,
                    double angularCreepSpeed,
                    boolean reverse) {
        this.downFieldInches = downFieldInches;
        this.xFieldInches = xFieldInches;
        this.turnRadiusInches = turnRadiusInches;
        this.maxLinearSpeed = maxLinearSpeed;
        this.maxAngularSpeedInDegsPerSec = maxAngularSpeedInDegsPerSec;
        this.linCreepSpeed = linCreepSpeed;
        this.angCreepSpeedInDegsPerSec = angularCreepSpeed;
        this.reverse = reverse;
    }

    public Waypoint(double downFieldInches,
                    double xFieldInches,
                    double turnRadiusInches,
                    boolean reverse,
                    boolean finalHeading) {
        this.downFieldInches = downFieldInches;
        this.xFieldInches = xFieldInches;
        this.turnRadiusInches = turnRadiusInches;
        this.reverse = reverse;
        this.finalHeading = finalHeading;
    }
}
