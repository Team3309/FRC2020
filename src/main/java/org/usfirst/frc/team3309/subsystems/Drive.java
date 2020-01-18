package org.usfirst.frc.team3309.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import org.usfirst.frc.team3309.Constants;
import org.usfirst.frc.team3309.lib.util.DriveSignal;

public class Drive {
    /*
     * Declare class members
     */
    private WPI_TalonFX leftFalconMaster;
    private WPI_TalonFX leftFalconSlave;
    private WPI_TalonFX rightFalconMaster;
    private WPI_TalonFX rightFalconSlave;

    public Drive() {
        /*
         * Initialize class members
         */
        leftFalconMaster = new WPI_TalonFX(Constants.DRIVE_LEFT_MASTER_FALCON_ID);
        leftFalconSlave = new WPI_TalonFX(Constants.DRIVE_LEFT_SLAVE_FALCON_ID);
        rightFalconMaster = new WPI_TalonFX(Constants.DRIVE_LEFT_MASTER_FALCON_ID);
        rightFalconSlave = new WPI_TalonFX(Constants.DRIVE_LEFT_SLAVE_FALCON_ID);

        configMaster(leftFalconMaster);
        configMaster(rightFalconMaster);
        configSlave(leftFalconSlave, leftFalconMaster);
        configSlave(rightFalconSlave, rightFalconMaster);
    }

    public void setLeftRight (ControlMode mode, DriveSignal signal) {
        setLeftRight(mode, signal.getLeft(), signal.getRight());
    }

    public void setLeftRight (ControlMode mode, double left, double right) {
        setLeft(mode, left);
        setRight(mode, right);
    }

    public void setLeft (ControlMode mode, double value) {
        leftFalconMaster.set(mode, value);
    }

    public void setRight (ControlMode mode, double value) {
        rightFalconMaster.set(mode, value);
    }

    private void configMaster(WPI_TalonFX talon) {
        talon.configFactoryDefault();
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        talon.configClosedloopRamp(Constants.DRIVE_CLOSED_LOOP_RAMP_RATE);
        talon.configOpenloopRamp(Constants.DRIVE_OPEN_LOOP_RAMP_RATE, 10);

        talon.config_kP(Constants.kDriveVelocitySlot, Constants.kDriveVelocityP, 10);
        talon.config_kD(Constants.kDriveVelocitySlot, Constants.kDriveVelocityD, 10);
        talon.config_kF(Constants.kDriveVelocitySlot, Constants.kDriveVelocityF, 10);

        talon.config_kP(Constants.kDrivePositionSlot, Constants.kDrivePositionP, 0);
        talon.config_kP(Constants.kDrivePositionSlot, Constants.kDrivePositionD, 0);

        talon.setNeutralMode(NeutralMode.Brake);
        talon.setInverted(true);
        talon.setSensorPhase(false);
        //addChild(talon);
    }

    private void configSlave(WPI_TalonFX slave, WPI_TalonFX master) {
        slave.configFactoryDefault();
        slave.follow(master);
        slave.setNeutralMode(NeutralMode.Brake);
        slave.setInverted(InvertType.FollowMaster);
        //addChild(slave);
    }
}
