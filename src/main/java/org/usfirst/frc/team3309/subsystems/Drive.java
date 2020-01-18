package org.usfirst.frc.team3309.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.util.DriveSignal;
import org.usfirst.frc.team3309.Constants;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import org.usfirst.frc.team3309.util.DriveSignal;


public class Drive extends SubsystemBase {

    //Memory allocation in preparation for Drive object initialization.
    private WPI_TalonFX driveMasterLeft;
    private WPI_TalonFX driveSlaveLeft;
    private WPI_TalonFX driveMasterRight;
    private WPI_TalonFX driveSlaveRight;

    private Solenoid shifter;

    //Configuration methods ready-to-go when Drive gets initialized.
    private void configDriveMaster(WPI_TalonFX talon) {

        talon.configFactoryDefault();
        int deviceID = talon.getDeviceID();

        talon.configClosedloopRamp(Constants.DRIVE_CLOSED_LOOP_RAMP_RATE);
        talon.configOpenloopRamp(Constants.DRIVE_OPEN_LOOP_RAMP_RATE, 10);
        talon.config_kP(deviceID, Constants.kDriveVelocityP);
        talon.config_kD(deviceID, Constants.kDriveVelocityD);
        talon.config_kF(deviceID, Constants.kDriveVelocityF);

        talon.setNeutralMode(NeutralMode.Brake);
        talon.setInverted(true);
        talon.setSensorPhase(false);

    }

    private void configDriveSlave(WPI_TalonFX slave, WPI_TalonFX master) {

        slave.configFactoryDefault();
        slave.follow(master);
        slave.setNeutralMode(NeutralMode.Brake);
        slave.setInverted(InvertType.FollowMaster);

    }

    //Initializes a Drive object by filling aforementioned memory slots and configuring
    //associated motors.
    public Drive() {

        driveMasterLeft = new WPI_TalonFX(Constants.DRIVE_LEFT_MASTER_ID);
        driveSlaveLeft = new WPI_TalonFX(Constants.DRIVE_LEFT_SLAVE_ID);
        driveMasterRight = new WPI_TalonFX(Constants.DRIVE_RIGHT_MASTER_ID);
        driveSlaveRight = new WPI_TalonFX(Constants.DRIVE_RIGHT_SLAVE_ID);

        configDriveMaster(driveMasterLeft);
        configDriveSlave(driveSlaveLeft, driveMasterLeft);
        configDriveMaster(driveMasterRight);
        configDriveSlave(driveSlaveRight, driveMasterRight);

    }

    public double getLeftEncoderDistance() {
        return driveMasterLeft.getSelectedSensorPosition(0);
    }

    public double getRightEncoderDistance() {
        return -driveMasterRight.getSelectedSensorPosition(0);
    }

    //Converts encoder ticks to physical distance(inches).
    public double encoderCountsToInches(double counts) {
        return counts/Constants.DRIVE_ENCODER_COUNTS_PER_REV * (Math.PI*Constants.DRIVE_WHEEL_DIAMETER_INCHES);
    }

    //Converts physical distance(inches) to encoder ticks.
    public double inchesToEncoderCounts(double inches) {
        return inches*(Constants.DRIVE_ENCODER_COUNTS_PER_REV/(Math.PI*Constants.DRIVE_WHEEL_DIAMETER_INCHES));
    }

    public double getLeftEncoderVelocity() {
        return driveMasterLeft.getSelectedSensorVelocity(0);
    }

    public double getRightEncoderVelocity() {
        return -driveMasterRight.getSelectedSensorVelocity(0);
    }

    //Converts degrees/sec to encoder velocity (ticks/sec).
    public double degreesPerSecToEncoderVelocity(double degreesPerSecond) {
        return degreesPerSecond * Constants.ENCODER_COUNTS_PER_DEGREE;
    }

    //Sets gearing...
    public void setHighGear() { shifter.set(true); }
    public void setLowGear() { shifter.set(false); }

    //...and gets gearing.
    public boolean inHighGear() {
        if(shifter.get() == true) {
            return true;
        } else return false;
    }
    public boolean inLowGear() {
        if(!inHighGear()) {
            return true;
        } else return false;
    }

    //Sets left motor.
    private void setLeft(ControlMode mode, double left,
                         DemandType demandType, double leftFeedForward) {
        driveMasterLeft.set(mode, left, demandType, leftFeedForward);
    }

    //Sets right motor.
    private void setRight(ControlMode mode, double right,
                          DemandType demandType, double rightFeedforward) {
        driveMasterRight.set(mode, -right, demandType, -rightFeedforward);
    }

    //Sets each of the motors simultaneously.
    public void setLeftRight(ControlMode mode, double left, double right) {
        driveMasterLeft.set(mode, left);
        driveMasterRight.set(mode, -right);
    }

    //Sets each of the motors simultaneously with assigned constants.
    public void setLeftRight(ControlMode mode, DemandType demandType,
                             double left, double right,
                             double leftFeedforward, double rightFeedforward) {
        setLeft(mode, left, demandType, leftFeedforward);
        setRight(mode, right, demandType, rightFeedforward);
    }

    //Sets the motors according to a DriveSignal object.
    public void setLeftRight (ControlMode mode, DriveSignal signal) {
        setLeftRight(mode, signal.getLeft(), signal.getRight());
    }

    //Sets the neutral mode for Drive.
    public void setNeutralMode(NeutralMode mode) {
        driveMasterLeft.setNeutralMode(mode);
        driveMasterRight.setNeutralMode(mode);
    }

    //Completely resets the Drive subsystem.
    public void reset() {
        driveMasterRight.clearMotionProfileTrajectories();
        driveMasterLeft.clearMotionProfileTrajectories();
        driveMasterRight.setSelectedSensorPosition(0, 0, 0);
        driveMasterRight.setSelectedSensorPosition(0, 0, 0);
    }

    //Sends motor data to SmartDashboard.
    public void outputToDashboard() {
        SmartDashboard.putNumber("Drive <- power", driveMasterLeft.getMotorOutputPercent());
        SmartDashboard.putNumber("Drive -> power", driveMasterRight.getMotorOutputPercent());
        SmartDashboard.putNumber("<- encoder distance", getLeftEncoderDistance());
        SmartDashboard.putNumber("-> encoder distance", getRightEncoderDistance());
        SmartDashboard.putNumber("<- encoder velocity", getLeftEncoderVelocity());
        SmartDashboard.putNumber("-> encoder velocity", getRightEncoderVelocity());
        SmartDashboard.putNumber("Left Drive Master current",
                Robot.pdp.getCurrent(Constants.kLeftDriveMasterPdpChannel));
        SmartDashboard.putNumber("Left Drive Slave current",
                Robot.pdp.getCurrent(Constants.kLeftDriveSlavePdpChannel));
        SmartDashboard.putNumber("Right Drive Master current",
                Robot.pdp.getCurrent(Constants.kRightDriveMasterPdpChannel));
        SmartDashboard.putNumber("Right Drive Slave current",
                Robot.pdp.getCurrent(Constants.kRightDriveSlavePdpChannel));
    }


}
