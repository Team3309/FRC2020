package frc.robot.util;

import frc.robot.subsystems.DriveSubsystem;

public class Waypoint {
    public double downFieldInches = 0; //how far the waypoint is from the driver station
    public double xFieldInches = 0; //lateral position of the waypoint
    public double turnRadiusInches = 0; //centered on the vertices of the straight-line path, not the guide circles
    public boolean reverse = false;  // robot backs into waypoint

    //TODO: Tune these
    public double linCreepSpeed = 5; //Inches per second
    public double angCreepSpeedInDegsPerSec = 2;
    public double maxLinearSpeed = 40; //Inches per 100 milliseconds
    public double maxAngularSpeedInDegsPerSec = 50;
    public double linAccelerationInInchesPerSec2 = 80; //Inches per 100 milliseconds^2
    public double linDecelerationInInchesPerSec2 = 160; //Also in inches per 100 milliseconds^2
    public double angAccelerationInDegsPerSec2 = 100;
    public double angDecelerationInDegsPerSec2 = 100;
    public double angToleranceInEncoderCounts;
    public double angToleranceInInches;
    public double linToleranceInEncoderCounts;
    public double linToleranceInInches;



    public double maxLinSpeedEncoderCtsPer100ms;
    public double linAccelerationEncoderCtsPer100ms2;
    public double linDecelerationEncoderCtsPer100ms2;
    public double linCreepSpeedEncoderCtsPer100ms;
    public double maxAngSpeedEncoderCtsPer100ms;
    public double angAccelerationEncoderCtsPer100ms2;
    public double angDecelerationEncoderCtsPer100ms2;
    public double angCreepSpeedEncoderCtsPer100ms;

    public Waypoint() {
        initialize();
    }

    public Waypoint(double downFieldInches,
                    double xFieldInches,
                    double turnRadiusInches,
                    boolean reverse) {
        this.downFieldInches = downFieldInches;
        this.xFieldInches = xFieldInches;
        this.turnRadiusInches = turnRadiusInches;
        this.reverse = reverse;

        initialize();
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

        initialize();
    }

    private void initialize () {
        maxLinSpeedEncoderCtsPer100ms = DriveSubsystem.inchesPerSecondToEncoderVelocity(maxLinearSpeed);
        linAccelerationEncoderCtsPer100ms2 = DriveSubsystem.inchesPerSecondToEncoderVelocity(linAccelerationInInchesPerSec2);
        linDecelerationEncoderCtsPer100ms2 = DriveSubsystem.inchesPerSecondToEncoderVelocity(linDecelerationInInchesPerSec2);
        linCreepSpeedEncoderCtsPer100ms = DriveSubsystem.inchesPerSecondToEncoderVelocity(linCreepSpeed);
        maxAngSpeedEncoderCtsPer100ms = DriveSubsystem.degreesPerSecondToEncoderVelocity(maxAngularSpeedInDegsPerSec);
        angAccelerationEncoderCtsPer100ms2 = DriveSubsystem.degreesPerSecondToEncoderVelocity(angAccelerationInDegsPerSec2);
        angDecelerationEncoderCtsPer100ms2 = DriveSubsystem.degreesPerSecondToEncoderVelocity(angDecelerationInDegsPerSec2);
        angCreepSpeedEncoderCtsPer100ms = DriveSubsystem.degreesPerSecondToEncoderVelocity(angCreepSpeedInDegsPerSec);
    }
}
