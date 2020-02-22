package frc.robot.util;

import frc.robot.subsystems.DriveSubsystem;

public class Waypoint {
    private DriveSubsystem drive;
    public double downFieldInches = 0; //how far the waypoint is from the driver station
    public double xFieldInches = 0; //lateral position of the waypoint
    public double turnRadiusInches = 0; //centered on the vertices of the straight-line path, not the guide circles
    public boolean reverse = false;  // robot backs into waypoint

    //TODO: Tune these
    public double linCreepSpeed = 5; //Inches per second
    public double angCreepSpeed = 23;
    public double maxLinearSpeed = 40; //Inches per 100 milliseconds
    public double maxAngularSpeed = 50; //Degrees per 100 milliseconds
    public double linAccelerationInInchesPer100ms2 = 80; //Inches per 100 milliseconds^2
    public double linDecelerationInInchesPer100ms2 = 160; //Also in inches per 100 milliseconds^2
    public double angAccelerationInDegsPer100ms2 = 70;
    public double angDecelerationInDegsPer100ms2 = 70;
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
        drive = new DriveSubsystem();
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
                    double maxAngularSpeed,
                    double linCreepSpeed,
                    double angularCreepSpeed,
                    boolean reverse) {
        this.downFieldInches = downFieldInches;
        this.xFieldInches = xFieldInches;
        this.turnRadiusInches = turnRadiusInches;
        this.maxLinearSpeed = maxLinearSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
        this.linCreepSpeed = linCreepSpeed;
        this.angCreepSpeed = angularCreepSpeed;
        this.reverse = reverse;

        initialize();
    }

    private void initialize () {
        maxLinSpeedEncoderCtsPer100ms = drive.inchesPerSecondToEncoderVelocity(maxLinearSpeed);
        linAccelerationEncoderCtsPer100ms2 = drive.inchesPerSecondToEncoderVelocity(linAccelerationInInchesPer100ms2);
        linDecelerationEncoderCtsPer100ms2 = drive.inchesPerSecondToEncoderVelocity(linDecelerationInInchesPer100ms2);
        linCreepSpeedEncoderCtsPer100ms = drive.inchesPerSecondToEncoderVelocity(linCreepSpeed);
        maxAngSpeedEncoderCtsPer100ms = drive.degreesPerSecToEncoderVelocity(maxAngularSpeed);
        angAccelerationEncoderCtsPer100ms2 = drive.degreesPerSecToEncoderVelocity(angAccelerationInDegsPer100ms2);
        angDecelerationEncoderCtsPer100ms2 = drive.degreesPerSecToEncoderVelocity(angDecelerationInDegsPer100ms2);
        angCreepSpeedEncoderCtsPer100ms = drive.degreesPerSecToEncoderVelocity(angCreepSpeed);
    }
}
