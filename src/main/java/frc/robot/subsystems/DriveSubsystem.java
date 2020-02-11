package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.util.DriveSignal;

public class DriveSubsystem extends SubsystemBase {

    private WPI_TalonFX driveMasterLeft;
    private WPI_TalonFX driveSlaveLeft;
    private WPI_TalonFX driveMasterRight;
    private WPI_TalonFX driveSlaveRight;

     /**---------------------------------------------------------------------------------------------------------------\
     * Initializes a Drive object by initializing the class member variables and configuring the new TalonFX objects.
     *
     \----------------------------------------------------------------------------------------------------------------*/
     public DriveSubsystem() {

         if (Config.isDriveInstalled) {
             driveMasterLeft = new WPI_TalonFX(Config.DriveLeftMasterID);
             driveSlaveLeft = new WPI_TalonFX(Config.DriveLeftSlaveID);
             driveMasterRight = new WPI_TalonFX(Config.DriveRightMasterID);
             driveSlaveRight = new WPI_TalonFX(Config.DriveRightSlaveID);

             configDriveMaster(driveMasterLeft);
             configDriveSlave(driveSlaveLeft, driveMasterLeft);
             configDriveMaster(driveMasterRight);
             configDriveSlave(driveSlaveRight, driveMasterRight);
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

        talon.configClosedloopRamp(Config.driveClosedLoopRampRate);
        talon.configOpenloopRamp(Config.driveOpenLoopRampRate, 10);
        talon.config_kP(deviceID, Config.driveVelocityP);
        talon.config_kI(0, Config.driveVelocityI, 10);
        talon.config_IntegralZone(0, Config.driveVelocityIntegralZone, 10);
        talon.config_kD(deviceID, Config.driveVelocityD);
        talon.config_kF(deviceID, Config.driveVelocityF);

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
    public double getLeftEncoderPosition() {
        if (Config.isDriveInstalled) {
            return driveMasterLeft.getSelectedSensorPosition(0);
        }
        return 0;
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Finds the current position of the right master encoder.
     *
     * @return The number of encoder counts away from the position at configuration.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public double getRightEncoderPosition() {
        if (Config.isDriveInstalled) {
            return -driveMasterRight.getSelectedSensorPosition(0);
        }
        return 0;
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Finds the current velocity of the left master encoder in counts per 100 milliseconds.
     *
     * @return The current velocity of the motor.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public double getLeftEncoderVelocity() {
        if (Config.isDriveInstalled) {
            return driveMasterLeft.getSelectedSensorVelocity(0);
        }
        return 0;
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Finds the current velocity of the left master encoder in counts per 100 milliseconds.
     *
     * @return The current velocity of the motor.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public double getRightEncoderVelocity() {
        if (Config.isDriveInstalled) {
            return -driveMasterRight.getSelectedSensorVelocity(0);
        }
        return 0;
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Converts degrees/sec to encoder velocity (counts/sec).
     *
     * @param degreesPerSecond - the velocity in degrees to be converted to encoder counts.
     * @return The encoder velocity in encoder counts.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public double degreesPerSecToEncoderVelocity(double degreesPerSecond) {
        return degreesPerSecond * Config.EncoderCountsPerDegree;
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Programs both motors simultaneously to move.
     *
     * @param mode - How the motors will move. See Cross The Road Electronics documentation for more information.
     * @param left - The value to which the left motor will be set.
     * @param right - The value to which the right motor will be set.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void setLeftRight(ControlMode mode, double left, double right) {
        if (Config.isDriveInstalled) {
            driveMasterLeft.set(mode, left);
            driveMasterRight.set(mode, -right);
        }
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Programs the motors to move according to the left and right constants of a certain DriveSignal object.
     *
     * @param mode - How the motors will move. See Cross The Road Electronics documentation for more information.
     * @param signal - The DriveSignal object which will serve to provide the left and right motor values.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void setLeftRight(ControlMode mode, DriveSignal signal) {
        setLeftRight(mode, signal.GetLeft(), signal.GetRight());
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Clears all drive encoder and PID data from before the method was called.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    public void reset() {
        if (Config.isDriveInstalled) {
            driveMasterRight.clearMotionProfileTrajectories();
            driveMasterLeft.clearMotionProfileTrajectories();
            driveMasterRight.setSelectedSensorPosition(0, 0, 0);
            driveMasterRight.setSelectedSensorPosition(0, 0, 0);
        }
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        SmartDashboard.putNumber("Drive left power", driveMasterLeft.getMotorOutputPercent());
        SmartDashboard.putNumber("Drive right power", -driveMasterRight.getMotorOutputPercent());
        SmartDashboard.putNumber("Drive left position", getLeftEncoderPosition());
        SmartDashboard.putNumber("Drive right position", getRightEncoderPosition());
        SmartDashboard.putNumber("Drive left velocity", getLeftEncoderVelocity());
        SmartDashboard.putNumber("Drive right velocity", getRightEncoderVelocity());
        SmartDashboard.putNumber("Drive left 1 current", Robot.pdp.getCurrent(Config.DriveLeftMasterPdpChannel));
        SmartDashboard.putNumber("Drive left 2 current", Robot.pdp.getCurrent(Config.DriveLeftSlavePdpChannel));
        SmartDashboard.putNumber("Drive right 1 current", Robot.pdp.getCurrent(Config.DriveRightMasterPdpChannel));
        SmartDashboard.putNumber("Drive right 2 current", Robot.pdp.getCurrent(Config.DriveRightSlavePdpChannel));
    }
}
