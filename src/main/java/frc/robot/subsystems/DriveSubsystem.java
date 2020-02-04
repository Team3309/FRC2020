package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DriveSignal;

public class DriveSubsystem extends SubsystemBase {

    public enum driveState {
        nothing,
        drivingStraight,
        spinTurning,
        mobileTurning
    }
    //Memory allocation in preparation for Drive object initialization.
    private WPI_TalonFX driveMasterLeft;
    private WPI_TalonFX driveSlaveLeft;
    private WPI_TalonFX driveMasterRight;
    private WPI_TalonFX driveSlaveRight;

    private Solenoid shifter;

    //Initializes a Drive object by initializing the class member variables and configuring the new TalonFX objects.
    public DriveSubsystem() {

        driveMasterLeft = new WPI_TalonFX(Constants.DRIVE_LEFT_MASTER_ID);
        driveSlaveLeft = new WPI_TalonFX(Constants.DRIVE_LEFT_SLAVE_ID);
        driveMasterRight = new WPI_TalonFX(Constants.DRIVE_RIGHT_MASTER_ID);
        driveSlaveRight = new WPI_TalonFX(Constants.DRIVE_RIGHT_SLAVE_ID);

        configDriveMaster(driveMasterLeft);
        configDriveSlave(driveSlaveLeft, driveMasterLeft);
        configDriveMaster(driveMasterRight);
        configDriveSlave(driveSlaveRight, driveMasterRight);

    }

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

    public double GetLeftEncoderDistance() {
        return driveMasterLeft.getSelectedSensorPosition(0);
    }

    public double GetRightEncoderDistance() {
        return -driveMasterRight.getSelectedSensorPosition(0);
    }

    public double GetLeftEncoderVelocity() {
        return driveMasterLeft.getSelectedSensorVelocity(0);
    }

    public double GetRightEncoderVelocity() {
        return -driveMasterRight.getSelectedSensorVelocity(0);
    }

    //Converts degrees/sec to encoder velocity (ticks/sec).
    public double DegreesPerSecToEncoderVelocity(double degreesPerSecond) {
        return degreesPerSecond * Constants.ENCODER_COUNTS_PER_DEGREE;
    }

    //Sets gearing...
    public void SetHighGear() { shifter.set(true); }
    public void SetLowGear() { shifter.set(false); }

    //...and gets gearing.
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

    //Sets left motor.
    private void SetLeft(ControlMode mode, double left,
                         DemandType demandType, double leftFeedForward) {
        driveMasterLeft.set(mode, left, demandType, leftFeedForward);
    }

    //Sets right motor.
    private void SetRight(ControlMode mode, double right,
                          DemandType demandType, double rightFeedforward) {
        driveMasterRight.set(mode, -right, demandType, -rightFeedforward);
    }

    //Sets each of the motors simultaneously.
    public void SetLeftRight(ControlMode mode, double left, double right) {
        driveMasterLeft.set(mode, left);
        driveMasterRight.set(mode, -right);
    }

    //Sets each of the motors simultaneously with assigned constants.
    public void SetLeftRight(ControlMode mode, DemandType demandType,
                             double left, double right,
                             double leftFeedforward, double rightFeedforward) {
        SetLeft(mode, left, demandType, leftFeedforward);
        SetRight(mode, right, demandType, rightFeedforward);
    }

    //Sets the motors according to a DriveSignal object.
    public void SetLeftRight(ControlMode mode, DriveSignal signal) {
        SetLeftRight(mode, signal.GetLeft(), signal.GetRight());
    }

    //Sets the neutral mode for Drive.
    public void SetNeutralMode(NeutralMode mode) {
        driveMasterLeft.setNeutralMode(mode);
        driveMasterRight.setNeutralMode(mode);
    }

    //Completely resets the Drive subsystem.
    public void Reset() {
        driveMasterRight.clearMotionProfileTrajectories();
        driveMasterLeft.clearMotionProfileTrajectories();
        driveMasterRight.setSelectedSensorPosition(0, 0, 0);
        driveMasterRight.setSelectedSensorPosition(0, 0, 0);
    }
    //Sends motor data to SmartDashboard.
    public void OutputToDashboard() {
        SmartDashboard.putNumber("Drive <- power", driveMasterLeft.getMotorOutputPercent());
        SmartDashboard.putNumber("Drive -> power", driveMasterRight.getMotorOutputPercent());
        SmartDashboard.putNumber("<- encoder distance", GetLeftEncoderDistance());
        SmartDashboard.putNumber("-> encoder distance", GetRightEncoderDistance());
        SmartDashboard.putNumber("<- encoder velocity", GetLeftEncoderVelocity());
        SmartDashboard.putNumber("-> encoder velocity", GetRightEncoderVelocity());
    }


}
