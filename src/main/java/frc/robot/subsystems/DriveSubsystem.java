package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.util.DriveSignal;

public class DriveSubsystem extends SubsystemBase {


    //Memory allocation in preparation for Drive object initialization.-------------------------------------------------
    private RobotContainer robotContainer;
    private WPI_TalonFX driveMasterLeft;
    private WPI_TalonFX driveSlaveLeft;
    private WPI_TalonFX driveMasterRight;
    private WPI_TalonFX driveSlaveRight;
    private Solenoid shifter;

    //Initializes a Drive object by initializing the class member variables and configuring the new TalonFX objects.----
    public DriveSubsystem(RobotContainer container) {

        robotContainer = container;
        driveMasterLeft = new WPI_TalonFX(Config.DriveLeftMasterID);
        driveSlaveLeft = new WPI_TalonFX(Config.DriveLeftSlaveID);
        driveMasterRight = new WPI_TalonFX(Config.DriveRightMasterID);
        driveSlaveRight = new WPI_TalonFX(Config.DriveRightSlaveID);

        configDriveMaster(driveMasterLeft);
        configDriveSlave(driveSlaveLeft, driveMasterLeft);
        configDriveMaster(driveMasterRight);
        configDriveSlave(driveSlaveRight, driveMasterRight);

    }

    //Configuration methods ready-to-go when Drive gets initialized.----------------------------------------------------
    private void configDriveMaster(WPI_TalonFX talon) {

        talon.configFactoryDefault();
        int deviceID = talon.getDeviceID();

        talon.configClosedloopRamp(Config.DriveClosedLoopRampRate);
        talon.configOpenloopRamp(Config.DriveOpenLoopRampRate, 10);
        talon.config_kP(deviceID, Config.DriveVelocityP);
        talon.config_kD(deviceID, Config.DriveVelocityD);
        talon.config_kF(deviceID, Config.DriveVelocityF);

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

    public double GetLeftEncoderPosition() {
        return driveMasterLeft.getSelectedSensorPosition(0);
    }

    public double GetRightEncoderPosition() {
        return -driveMasterRight.getSelectedSensorPosition(0);
    }

    public double GetLeftEncoderVelocity() {
        return driveMasterLeft.getSelectedSensorVelocity(0);
    }

    public double GetRightEncoderVelocity() {
        return -driveMasterRight.getSelectedSensorVelocity(0);
    }

    //Converts degrees/sec to encoder velocity (ticks/sec).-------------------------------------------------------------
    public double DegreesPerSecToEncoderVelocity(double degreesPerSecond) {
        return degreesPerSecond * Config.EncoderCountsPerDegree;
    }

    //Sets gearing...---------------------------------------------------------------------------------------------------
    public void SetHighGear() { shifter.set(true); }
    public void SetLowGear() { shifter.set(false); }

    //...and gets gearing.----------------------------------------------------------------------------------------------
    public boolean InHighGear() {
        if(shifter.get() == true) {
            return true;
        } else return false;
    }

    public boolean InLowGear() {
        if(!InHighGear()) {
            return true;
        } else return false;
    }

    //Sets left motor.--------------------------------------------------------------------------------------------------
    private void SetLeft(ControlMode mode, double left,
                         DemandType demandType, double leftFeedForward) {
        driveMasterLeft.set(mode, left, demandType, leftFeedForward);
    }

    //Sets right motor.-------------------------------------------------------------------------------------------------
    private void SetRight(ControlMode mode, double right,
                          DemandType demandType, double rightFeedforward) {
        driveMasterRight.set(mode, -right, demandType, -rightFeedforward);
    }

    //Sets each of the motors simultaneously.---------------------------------------------------------------------------
    public void SetLeftRight(ControlMode mode, double left, double right) {
        driveMasterLeft.set(mode, left);
        driveMasterRight.set(mode, -right);
    }

    //Sets each of the motors simultaneously with assigned constants.---------------------------------------------------
    public void SetLeftRight(ControlMode mode, DemandType demandType,
                             double left, double right,
                             double leftFeedforward, double rightFeedforward) {
        SetLeft(mode, left, demandType, leftFeedforward);
        SetRight(mode, right, demandType, rightFeedforward);
    }

    //Sets the motors according to a DriveSignal object.----------------------------------------------------------------
    public void SetLeftRight(ControlMode mode, DriveSignal signal) {
        SetLeftRight(mode, signal.GetLeft(), signal.GetRight());
    }

    //Sets the neutral mode for Drive.----------------------------------------------------------------------------------
    public void SetNeutralMode(NeutralMode mode) {
        driveMasterLeft.setNeutralMode(mode);
        driveMasterRight.setNeutralMode(mode);
    }

    //Completely resets the Drive subsystem.----------------------------------------------------------------------------
    public void Reset() {
        driveMasterRight.clearMotionProfileTrajectories();
        driveMasterLeft.clearMotionProfileTrajectories();
        driveMasterRight.setSelectedSensorPosition(0, 0, 0);
        driveMasterRight.setSelectedSensorPosition(0, 0, 0);
    }

    //Sends motor data to SmartDashboard.-------------------------------------------------------------------------------
    public void OutputToDashboard() {
        SmartDashboard.putNumber("Drive left power", driveMasterLeft.getMotorOutputPercent());
        SmartDashboard.putNumber("Drive right power", -driveMasterRight.getMotorOutputPercent());
        SmartDashboard.putNumber("Drive left position", GetLeftEncoderPosition());
        SmartDashboard.putNumber("Drive right position", GetRightEncoderPosition());
        SmartDashboard.putNumber("Drive left velocity", GetLeftEncoderVelocity());
        SmartDashboard.putNumber("Drive right velocity", GetRightEncoderVelocity());
        SmartDashboard.putNumber("Drive left 1 current", robotContainer.GetCurrent(Config.DriveLeftMasterPdpChannel));
        SmartDashboard.putNumber("Drive left 2 current", robotContainer.GetCurrent(Config.DriveLeftSlavePdpChannel));
        SmartDashboard.putNumber("Drive right 1 current", robotContainer.GetCurrent(Config.DriveRightMasterPdpChannel));
        SmartDashboard.putNumber("Drive right 2 current", robotContainer.GetCurrent(Config.DriveRightSlavePdpChannel));
    }


}
