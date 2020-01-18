package org.usfirst.frc.team3309.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3309.util.DriveSignal;
import org.usfirst.frc.team3309.Constants;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import org.usfirst.frc.team3309.util.DriveSignal;


public class Drive {

    /*
-     * Declare class members
-     */
    private WPI_TalonFX leftFalconMaster;
    private WPI_TalonFX leftFalconSlave;
    private WPI_TalonFX rightFalconMaster;
    private WPI_TalonFX rightFalconSlave;

    private WPI_TalonFX driveMasterLeft;
    private WPI_TalonFX driveSlaveLeft1, driveSlaveLeft2, driveSlaveLeft3;
    private WPI_TalonFX driveMasterRight;
    private WPI_TalonFX driveSlaveRight1, driveSlaveRight2, driveSlaveRight3;

    private Solenoid shifter;

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

    public Drive() {

        driveMasterLeft = new WPI_TalonFX(Constants.F500_D_M_L);
        driveSlaveLeft1 = new WPI_TalonFX(Constants.F500_D_S_L1);
        driveSlaveLeft2 = new WPI_TalonFX(Constants.F500_D_S_L2);
        driveSlaveLeft3 = new WPI_TalonFX(Constants.F500_D_S_L3);
        driveMasterRight = new WPI_TalonFX(Constants.F500_D_M_R);
        driveSlaveRight1 = new WPI_TalonFX(Constants.F500_D_S_R1);
        driveSlaveRight2 = new WPI_TalonFX(Constants.F500_D_S_R2);
        driveSlaveRight3 = new WPI_TalonFX(Constants.F500_D_S_R3);

        configDriveMaster(driveMasterLeft);
        configDriveSlave(driveSlaveLeft1, driveMasterLeft);
        configDriveSlave(driveSlaveLeft2, driveMasterLeft);
        configDriveSlave(driveSlaveLeft3, driveMasterLeft);
        configDriveMaster(driveMasterRight);
        configDriveSlave(driveSlaveRight1, driveMasterRight);
        configDriveSlave(driveSlaveRight2, driveMasterRight);
        configDriveSlave(driveSlaveRight3, driveMasterRight);

    }

    public double getLeftEncoderDistance() {
        return driveMasterLeft.getSelectedSensorPosition(0);
    }

    public double getRightEncoderDistance() {
        return -driveMasterRight.getSelectedSensorPosition(0);
    }

    public double encoderCountsToInches(double counts) {
        return counts/Constants.DRIVE_ENCODER_COUNTS_PER_REV * (Math.PI*Constants.DRIVE_WHEEL_DIAMETER_INCHES);
    }

    public double inchesToEncoderCounts(double inches) {
        return inches*(Constants.DRIVE_ENCODER_COUNTS_PER_REV/(Math.PI*Constants.DRIVE_WHEEL_DIAMETER_INCHES));
    }

    public double getLeftEncoderVelocity() {
        return driveMasterLeft.getSelectedSensorVelocity(0);
    }

    public double getRightEncoderVelocity() {
        return -driveMasterRight.getSelectedSensorVelocity(0);
    }

    public double degreesPerSecToEncoderVelocity(double degreesPerSecond) {
        return degreesPerSecond * Constants.ENCODER_COUNTS_PER_DEGREE;
    }

    public void setHighGear() { shifter.set(true); }
    public void setLowGear() { shifter.set(false); }
    public boolean inHighGear() {
        if(shifter.get() == true) {
            return true;
        } else return false;
    }
    public boolean inLowGear() {
        if(!inHighGear() == true) {
            return true;
        } else return false;
    }

    private void setLeft(ControlMode mode, double left,
                         DemandType demandType, double leftFeedForward) {
        driveMasterLeft.set(mode, left, demandType, leftFeedForward);
    }
    private void setRight(ControlMode mode, double right,
                          DemandType demandType, double rightFeedforward) {
        driveMasterRight.set(mode, -right, demandType, -rightFeedforward);
    }

    public void setLeftRight(ControlMode mode, double left, double right) {
        driveMasterLeft.set(mode, left);
        driveMasterRight.set(mode, -right);
    }

    public void setLeftRight(ControlMode mode, DemandType demandType,
                             double left, double right,
                             double leftFeedforward, double rightFeedforward) {
        setLeft(mode, left, demandType, leftFeedforward);
        setRight(mode, right, demandType, rightFeedforward);
    }

    public void setLeftRight (ControlMode mode, DriveSignal signal) {
        setLeftRight(mode, signal.getLeft(), signal.getRight());
    }

    public void setNeutralMode(NeutralMode mode) {
        driveMasterLeft.setNeutralMode(mode);
        driveMasterRight.setNeutralMode(mode);
    }

    public void reset() {
        driveMasterRight.clearMotionProfileTrajectories();
        driveMasterLeft.clearMotionProfileTrajectories();
        driveMasterRight.setSelectedSensorPosition(0, 0, 0);
        driveMasterRight.setSelectedSensorPosition(0, 0, 0);
    }

    public void outputToDashboard() {
        SmartDashboard.putNumber("Drive <- power", driveMasterLeft.getMotorOutputPercent());
        SmartDashboard.putNumber("Drive -> power", driveMasterRight.getMotorOutputPercent());
        SmartDashboard.putNumber("<- encoder distance", getLeftEncoderDistance());
        SmartDashboard.putNumber("-> encoder distance", getRightEncoderDistance());
        SmartDashboard.putNumber("<- encoder velocity", getLeftEncoderVelocity());
        SmartDashboard.putNumber("-> encoder velocity", getRightEncoderVelocity());
    }


}
