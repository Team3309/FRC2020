package frc.robot.util;

import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Config;

public class IMU3309{

    private ADIS16470_IMU adis;
    private Timer timer;

    public IMU3309() {
        adis = new ADIS16470_IMU(Config.imuAxis, SPI.Port.kOnboardCS0, Config.imuCalibrationTime);
        timer = new Timer();
        timer.start();
        adis.calibrate();
    }

    public void reset() {
        adis.reset();
        timer.reset();
    }

    public void close() {
        adis.close();
    }

    public void calibrate() {
        adis.calibrate();
        timer.reset();
    }

    public void configCalibrationTime(ADIS16470_IMU.ADIS16470CalibrationTime calTime) {
        adis.configCalTime(calTime);
    }

    public double getInstantAccelerationX() {
        return adis.getAccelInstantX();
    }

    public double getInstantAccelerationY() {
        return adis.getAccelInstantY();
    }

    public double getInstantAccelerationZ() {
        return adis.getAccelInstantZ();
    }

    public double getInstantGyroX() {
        return adis.getGyroInstantX();
    }

    public double getInstantGyroY() {
        return adis.getGyroInstantY();
    }

    public double getInstantGyroZ() {
        return adis.getGyroInstantZ();
    }

    public double getXComplementaryAngle() {
        return adis.getXComplementaryAngle();
    }

    public double getYComplementaryAngle() {
        return adis.getYComplementaryAngle();

    }

    public double getXFilteredAccelerationAngle() {
        return adis.GetXFilteredAccelAngle();
    }

    public double getYFilteredAccelerationAngle() {
        return adis.GetYFilteredAccelAngle();
    }

    public double getAngle() {
        // TODO: subtract calibration time from the timer
        return adis.getAngle() - (timer.get() * Config.IMUDriftConstant);
    }

    public double getRate() {
        return adis.getRate();
    }

    public int setYawAxis(ADIS16470_IMU.IMUAxis axis) {
        return adis.setYawAxis(axis);
    }

    public ADIS16470_IMU.IMUAxis getYawAxis() {
        return adis.getYawAxis();
    }

}
