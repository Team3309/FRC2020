package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.util.DriveSignal;
import frc.robot.util.IMU3309;

import static frc.robot.util.UnitConversions.inchesToEncoderCounts;

public class DriveSubsystem extends SubsystemBase {

    private WPI_TalonFX driveMasterLeft;
    private WPI_TalonFX driveSlaveLeft;
    private WPI_TalonFX driveMasterRight;
    private WPI_TalonFX driveSlaveRight;
    private IMU3309 imu;
    private Timer ctrlTimer;

    private WPI_TalonSRX driveMasterLeft2019, driveMasterRight2019;

     /**----------------------------------------------------------------------------------------------------------------
     * Initializes a Drive object by initializing the class member variables and configuring the new TalonFX objects.
     *
     */
     public DriveSubsystem() {
         if (Config.isDriveInstalled) {
             //non specific robot configuration
             ctrlTimer = new Timer();
             ctrlTimer.start();
             if (Config.isIMUInstalled) {
                 imu = new IMU3309();
             }
             //2019 specific configuration
             if (Config.currentRobot == Config.RobotModel.Practice2019) {
                 driveMasterLeft2019 = new WPI_TalonSRX(Config.driveLeftMasterID);
                 WPI_VictorSPX driveLeftSlave1 = new WPI_VictorSPX(Config.driveLeftSlaveID2019_1);
                 WPI_VictorSPX driveLeftSlave2 = new WPI_VictorSPX(Config.driveLeftSlaveID2019_2);
                 driveMasterRight2019 = new WPI_TalonSRX(Config.driveRightMasterID);
                 WPI_VictorSPX driveRightSlave1 = new WPI_VictorSPX(Config.driveRightSlaveID2019_1);
                 WPI_VictorSPX driveRightSlave2 = new WPI_VictorSPX(Config.driveRightSlaveID2019_2);

                 //Configure Left Side of Drive
                 configMaster2019(driveMasterLeft2019);
                 configSlave2019(driveLeftSlave1, driveMasterLeft2019);
                 configSlave2019(driveLeftSlave2, driveMasterLeft2019);

                 //Configure Right Side of Drive
                 configMaster2019(driveMasterRight2019);
                 configSlave2019(driveRightSlave1, driveMasterRight2019);
                 configSlave2019(driveRightSlave2, driveMasterRight2019);
             } else {
                 driveMasterLeft = new WPI_TalonFX(Config.driveLeftMasterID);
                 driveSlaveLeft = new WPI_TalonFX(Config.driveLeftSlaveID);
                 driveMasterRight = new WPI_TalonFX(Config.driveRightMasterID);
                 driveSlaveRight = new WPI_TalonFX(Config.driveRightSlaveID);



                 configDriveMaster(driveMasterLeft);
                 configDriveSlave(driveSlaveLeft, driveMasterLeft);
                 configDriveMaster(driveMasterRight);
                 configDriveSlave(driveSlaveRight, driveMasterRight);


             }
         }
    }


    private void configMaster2019(WPI_TalonSRX talon) {


        talon.configFactoryDefault();
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        talon.configClosedloopRamp(Config.driveClosedLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.configOpenloopRamp(Config.driveOpenLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.config_kP(0, Config.driveVelocityP, Config.motorControllerConfigTimeoutMs);
        talon.config_IntegralZone(0, Config.driveVelocityIntegralZone, Config.motorControllerConfigTimeoutMs);
        talon.config_kD(0, Config.driveVelocityD, Config.motorControllerConfigTimeoutMs);
        talon.config_kF(0, Config.driveVelocityF, Config.motorControllerConfigTimeoutMs);


        talon.setNeutralMode(NeutralMode.Brake);
        talon.setInverted(true);
        talon.setSensorPhase(false);
    }



    private void configSlave2019(WPI_VictorSPX slave, WPI_TalonSRX master) {
        slave.configFactoryDefault();
        slave.follow(master);
        slave.setNeutralMode(NeutralMode.Brake);
        slave.setInverted(InvertType.FollowMaster);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Configures a master motor with preset PID constants.
     *
     * @param talon - the Falcon motor to be configured.
     *
     */
    private void configDriveMaster(WPI_TalonFX talon) {

        talon.configFactoryDefault();

        talon.configClosedloopRamp(Config.driveClosedLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.configOpenloopRamp(Config.driveOpenLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.config_kP(0, Config.driveVelocityP, Config.motorControllerConfigTimeoutMs);
        talon.config_kI(0, Config.driveVelocityI, Config.motorControllerConfigTimeoutMs);
        talon.config_IntegralZone(0, Config.driveVelocityIntegralZone, Config.motorControllerConfigTimeoutMs);
        talon.config_kD(0, Config.driveVelocityD, Config.motorControllerConfigTimeoutMs);
        talon.config_kF(0, Config.driveVelocityF, Config.motorControllerConfigTimeoutMs);

        talon.setNeutralMode(NeutralMode.Brake);
        talon.setInverted(true);
        talon.setSensorPhase(false);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Configures a slave motor with the same constants as the master motor and sets it to do what the master motor is
     * doing.
     *
     * @param slave - the motor to follow master.
     * @param master - the motor according to which slave is configured.
     *
     */
    private void configDriveSlave(WPI_TalonFX slave, WPI_TalonFX master) {
        slave.configFactoryDefault();
        slave.follow(master);
        slave.setNeutralMode(NeutralMode.Brake);
        slave.setInverted(InvertType.FollowMaster);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Finds the current position of the left master encoder.
     *
     * @return The number of encoder counts away from the position at configuration.
     *
     */
    public double getLeftEncoderPosition() {
        if (Config.isDriveInstalled) {
            if (Config.currentRobot == Config.RobotModel.Practice2019) {
                return driveMasterLeft2019.getSelectedSensorPosition(0);
            } else {
                return driveMasterLeft.getSelectedSensorPosition(0);
            }
        }
        return 0;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Finds the current position of the right master encoder.
     *
     * @return The number of encoder counts away from the position at configuration.
     *
    */
    public double getRightEncoderPosition() {
        if (Config.isDriveInstalled) {
            if (Config.currentRobot == Config.RobotModel.Practice2019) {
                return -driveMasterRight2019.getSelectedSensorPosition(0 );
            } else {
                return -driveMasterRight.getSelectedSensorPosition(0);
            }

        }
        return 0;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Finds the current velocity of the left master encoder in counts per 100 milliseconds.
     *
     * @return The current velocity of the motor.
     *
     */
    public double getLeftEncoderVelocity() {
        if (Config.isDriveInstalled) {
            if (Config.currentRobot == Config.RobotModel.Practice2019) {
                return driveMasterLeft2019.getSelectedSensorVelocity(0);
            } else {
                return driveMasterLeft.getSelectedSensorVelocity(0);
            }

        }
        return 0;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Finds the current velocity of the left master encoder in counts per 100 milliseconds.
     *
     * @return The current velocity of the motor.
     *
     */
    public double getRightEncoderVelocity() {
        if (Config.isDriveInstalled) {
            if (Config.currentRobot == Config.RobotModel.Practice2019) {
                return -driveMasterRight2019.getSelectedSensorVelocity();
            } else {
                return -driveMasterRight.getSelectedSensorVelocity(0);
            }
        }
        return 0;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Gets the current angular position away from its initialization position.
     *
     * @return The current angle of the robot.
     *
     */
    public double getAngularPosition() {
        if (Config.isIMUInstalled) {
            return imu.getAngle();
        } else return 0;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Gets the current angular velocity of the robot.
     *
     * @return The current angular velocity.
     *
     */
    public double getAngularVelocity() {
        if (Config.isIMUInstalled) {
            return imu.getRate();
        } else return 0;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Sets the angle of the IMU to zero.
     *
     */
    public void zeroImu() {
        if (Config.isIMUInstalled) {
            imu.reset();
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Converts degrees/sec to encoder velocity (counts/sec).
     *
     * @param degreesPerSecond - the velocity in degrees to be converted to encoder counts.
     * @return The encoder velocity in encoder counts.
     *
     */
    public double degreesPerSecToEncoderVelocity(double degreesPerSecond) {
        return degreesPerSecond * Config.encoderCountsPerDegree;
    }

    public double inchesPerSecondToEncoderVelocity(double inchesPerSecond) {
        return inchesToEncoderCounts(inchesPerSecond / 10.0 * 4096.0 / (Math.PI * Config.driveWheelDiameterInInches));
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs both motors simultaneously to move.
     *
     * @param mode - How the motors will move. See Cross The Road Electronics documentation for more information.
     * @param left - The value to which the left motor will be set.
     * @param right - The value to which the right motor will be set.
     *
     */
    public void setLeftRight(ControlMode mode, double left, double right) {
        if (Config.isDriveInstalled) {

            if (Config.currentRobot == Config.RobotModel.Practice2019) {
                driveMasterLeft2019.set(mode, left);
                driveMasterRight2019.set(mode, -right);
            } else {
                driveMasterLeft.set(mode, left);
                driveMasterRight.set(mode, -right);
            }

        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the motors to move according to the left and right constants of a certain DriveSignal object.
     *
     * @param mode - How the motors will move. See Cross The Road Electronics documentation for more information.
     * @param signal - The DriveSignal object which will serve to provide the left and right motor values.
     *
     */
    public void setLeftRight(ControlMode mode, DriveSignal signal) {
        setLeftRight(mode, signal.GetLeft(), signal.GetRight());
    }

    public void setArcade(ControlMode mode, double speed, double turn) {
        setLeftRight(mode, speed + turn, speed - turn);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Clears all drive encoder and PID data from before the method was called.
     *
     */
    public void reset() {
        if (Config.isDriveInstalled) {
            driveMasterRight.clearMotionProfileTrajectories();
            driveMasterLeft.clearMotionProfileTrajectories();
            driveMasterRight.setSelectedSensorPosition(0, 0, 0);
            driveMasterRight.setSelectedSensorPosition(0, 0, 0);
        }
    }

    public boolean getDebugMode() {
        return Config.isDebugMode;
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        if(Config.currentRobot == Config.RobotModel.Practice2019) {

            SmartDashboard.putNumber("Drive left power", driveMasterLeft2019.getMotorOutputPercent());
            SmartDashboard.putNumber("Drive right power", -driveMasterRight2019.getMotorOutputPercent());
        } else {
            SmartDashboard.putNumber("Drive left power", driveMasterLeft.getMotorOutputPercent());
            SmartDashboard.putNumber("Drive right power", -driveMasterRight.getMotorOutputPercent());
        }

        SmartDashboard.putNumber("Drive left position", getLeftEncoderPosition());
        SmartDashboard.putNumber("Drive right position", getRightEncoderPosition());
        SmartDashboard.putNumber("Drive left velocity", getLeftEncoderVelocity());
        SmartDashboard.putNumber("Drive right velocity", getRightEncoderVelocity());
        SmartDashboard.putNumber("Drive left 1 current", Robot.pdp.getCurrent(Config.driveLeftMasterPdpChannel));
        SmartDashboard.putNumber("Drive left 2 current", Robot.pdp.getCurrent(Config.driveLeftSlavePdpChannel));
        SmartDashboard.putNumber("Drive right 1 current", Robot.pdp.getCurrent(Config.driveRightMasterPdpChannel));
        SmartDashboard.putNumber("Drive right 2 current", Robot.pdp.getCurrent(Config.driveRightSlavePdpChannel));
        SmartDashboard.putNumber("Angular position", getAngularPosition());
        SmartDashboard.putNumber("Angular velocity", getAngularVelocity());
        SmartDashboard.putNumber("Current DriveSubsystem Time", ctrlTimer.get());
        SmartDashboard.putNumber("Drift in degs sec^-1", getAngularPosition()/ctrlTimer.get());
    }
}
