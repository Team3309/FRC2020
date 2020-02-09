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
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.DriveSignal;

import static frc.robot.Config.isDriveInstalled;

public class DriveSubsystem extends SubsystemBase {

    private WPI_TalonFX driveMasterLeft;
    private WPI_TalonFX driveSlaveLeft;
    private WPI_TalonFX driveMasterRight;
    private WPI_TalonFX driveSlaveRight;
    private Solenoid shifter;

     /**---------------------------------------------------------------------------------------------------------------\
     * Initializes a Drive object by initializing the class member variables and configuring the new TalonFX objects.
     *
     \----------------------------------------------------------------------------------------------------------------*/
     public DriveSubsystem() {

         if (isDriveInstalled) {
             driveMasterLeft = new WPI_TalonFX(Config.DriveLeftMasterID);
             driveSlaveLeft = new WPI_TalonFX(Config.DriveLeftSlaveID);
             driveMasterRight = new WPI_TalonFX(Config.DriveRightMasterID);
             driveSlaveRight = new WPI_TalonFX(Config.DriveRightSlaveID);

             configDriveMaster(driveMasterLeft);
             configDriveSlave(driveSlaveLeft, driveMasterLeft);
             configDriveMaster(driveMasterRight);
             configDriveSlave(driveSlaveRight, driveMasterRight);
             SetNeutralMode(NeutralMode.Brake);
         }
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Configures a master motor with preset PID constants.
     *
     * @param talon - the Falcon motor to be configured.
     *
     \----------------------------------------------------------------------------------------------------------------*/
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

     /**---------------------------------------------------------------------------------------------------------------\
     * Configures a slave motor with the same constants as the master motor and sets it to do what the master motor is
     * doing.
     *
     * @param slave - the motor to follow master.
     * @param master - the motor according to which slave is configured.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    private void configDriveSlave(WPI_TalonFX slave, WPI_TalonFX master) {
        slave.configFactoryDefault();
        slave.follow(master);
        slave.setNeutralMode(NeutralMode.Brake);
        slave.setInverted(InvertType.FollowMaster);
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Finds the current position of the left master encoder.
     *
     * @return The number of encoder counts away from the position at configuration.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public double GetLeftEncoderPosition() {
        return driveMasterLeft.getSelectedSensorPosition(0);
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Finds the current position of the right master encoder.
     *
     * @return The number of encoder counts away from the position at configuration.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public double GetRightEncoderPosition() {
        return -driveMasterRight.getSelectedSensorPosition(0);
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Finds the current velocity of the left master encoder in counts per 100 milliseconds.
     *
     * @return The current velocity of the motor.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public double GetLeftEncoderVelocity() {
        return driveMasterLeft.getSelectedSensorVelocity(0);
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Finds the current velocity of the left master encoder in counts per 100 milliseconds.
     *
     * @return The current velocity of the motor.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public double GetRightEncoderVelocity() {
        return -driveMasterRight.getSelectedSensorVelocity(0);
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Converts degrees/sec to encoder velocity (counts/sec).
     *
     * @param degreesPerSecond - the velocity in degrees to be converted to encoder counts.
     * @return The encoder velocity in encoder counts.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public double DegreesPerSecToEncoderVelocity(double degreesPerSecond) {
        return degreesPerSecond * Config.EncoderCountsPerDegree;
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Programs the left motor to move according to code instructions.
     *
     * @param mode - The desired control mode for the motor. See Cross The Road Electronics documentation for more
     *             information.
     * @param left - The value to which the left motor will be set.
     * @param demandType - *****
     * @param leftFeedForward - *****
     *
     \----------------------------------------------------------------------------------------------------------------*/
    private void SetLeft(ControlMode mode, double left,
                         DemandType demandType, double leftFeedForward) {
        driveMasterLeft.set(mode, left, demandType, leftFeedForward);
    }


     /**---------------------------------------------------------------------------------------------------------------\
     * Programs the right motor to move according to code instructions.
     *
     * @param mode - How the motor will move. See Cross The Road Electronics documentation for more information.
     * @param right - The value to which the right motor will be set.
     * @param demandType - *****
     * @param rightFeedForward - *****
     *
     \----------------------------------------------------------------------------------------------------------------*/
    private void SetRight(ControlMode mode, double right,
                          DemandType demandType, double rightFeedForward) {
        driveMasterRight.set(mode, -right, demandType, -rightFeedForward);
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Programs both motors simultaneously to move.
     *
     * @param mode - How the motors will move. See Cross The Road Electronics documentation for more information.
     * @param left - The value to which the left motor will be set.
     * @param right - The value to which the right motor will be set.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void SetLeftRight(ControlMode mode, double left, double right) {
        driveMasterLeft.set(mode, left);
        driveMasterRight.set(mode, -right);
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Programs each motor simultaneously to move according to certain control constants.
     *
     * @param mode - How the motors will move. See Cross The Road Electronics documentation for more information.
     * @param demandType - *****
     * @param left - The value to which the left motor will be set.
     * @param right - The value to which the right motor will be set.
     * @param leftFeedforward - *****
     * @param rightFeedforward - *****
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void SetLeftRight(ControlMode mode, DemandType demandType,
                             double left, double right,
                             double leftFeedforward, double rightFeedforward) {
        SetLeft(mode, left, demandType, leftFeedforward);
        SetRight(mode, right, demandType, rightFeedforward);
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Programs the motors to move according to the left and right constants of a certain DriveSignal object.
     *
     * @param mode - How the motors will move. See Cross The Road Electronics documentation for more information.
     * @param signal - The DriveSignal object which will serve to provide the left and right motor values.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void SetLeftRight(ControlMode mode, DriveSignal signal) {
        SetLeftRight(mode, signal.GetLeft(), signal.GetRight());
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Sets the default motor behavior for when the motors are neutral (when they are set to 0.)
     *
     * @param mode - The default motor behavior for the drive motors when set to 0.
     \----------------------------------------------------------------------------------------------------------------*/
    public void SetNeutralMode(NeutralMode mode) {
        driveMasterLeft.setNeutralMode(mode);
        driveMasterRight.setNeutralMode(mode);
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Clears all drive encoder and PID data from before the method was called.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void Reset() {
        driveMasterRight.clearMotionProfileTrajectories();
        driveMasterLeft.clearMotionProfileTrajectories();
        driveMasterRight.setSelectedSensorPosition(0, 0, 0);
        driveMasterRight.setSelectedSensorPosition(0, 0, 0);
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        SmartDashboard.putNumber("Drive left power", driveMasterLeft.getMotorOutputPercent());
        SmartDashboard.putNumber("Drive right power", -driveMasterRight.getMotorOutputPercent());
        SmartDashboard.putNumber("Drive left position", GetLeftEncoderPosition());
        SmartDashboard.putNumber("Drive right position", GetRightEncoderPosition());
        SmartDashboard.putNumber("Drive left velocity", GetLeftEncoderVelocity());
        SmartDashboard.putNumber("Drive right velocity", GetRightEncoderVelocity());
        SmartDashboard.putNumber("Drive left 1 current", Robot.pdp.getCurrent(Config.DriveLeftMasterPdpChannel));
        SmartDashboard.putNumber("Drive left 2 current", Robot.pdp.getCurrent(Config.DriveLeftSlavePdpChannel));
        SmartDashboard.putNumber("Drive right 1 current", Robot.pdp.getCurrent(Config.DriveRightMasterPdpChannel));
        SmartDashboard.putNumber("Drive right 2 current", Robot.pdp.getCurrent(Config.DriveRightSlavePdpChannel));
    }
}
