package frc.robot.util;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.UnitConversions;

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
    public double linAccelerationInchesPer100ms2 = 80; //Inches per 100 milliseconds^2
    public double linDecelerationInchesPer100ms2 = 160; //Also in inches per 100 milliseconds^2
    public double angAccelerationDegsPer100ms2 = 70;
    public double angDecelerationDegsPer100ms2 = 70;
    public double linToleranceEncoderCounts;
    public double linToleranceInches;

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
        linAccelerationEncoderCtsPer100ms2 = drive.inchesPerSecondToEncoderVelocity(linAccelerationInchesPer100ms2);
        linDecelerationEncoderCtsPer100ms2 = drive.inchesPerSecondToEncoderVelocity(linDecelerationInchesPer100ms2);
        linCreepSpeedEncoderCtsPer100ms = drive.inchesPerSecondToEncoderVelocity(linCreepSpeed);
        maxAngSpeedEncoderCtsPer100ms = drive.degreesPerSecToEncoderVelocity(maxAngularSpeed);
        angAccelerationEncoderCtsPer100ms2 = drive.degreesPerSecToEncoderVelocity(angAccelerationDegsPer100ms2);
        angDecelerationEncoderCtsPer100ms2 = drive.degreesPerSecToEncoderVelocity(angDecelerationDegsPer100ms2);
        angCreepSpeedEncoderCtsPer100ms = drive.degreesPerSecToEncoderVelocity(angCreepSpeed);
    }
}
