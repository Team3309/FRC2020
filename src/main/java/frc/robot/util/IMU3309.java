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

    public double getCalibrationTime() {
        double calTime;

        switch (Config.imuCalibrationTime) {

            case _32ms: calTime = 0.032;
            break;
            case _64ms: calTime =  0.064;
            break;
            case _128ms: calTime = 0.128;
            break;
            case _256ms: calTime = 0.256;
            break;
            case _512ms: calTime = 0.512;
            break;
            case _1s: calTime = 1.0;
            break;
            case _2s: calTime = 2.0;
            break;
            case _8s: calTime = 8.0;
            break;
            case _16s: calTime = 16.0;
            break;
            case _32s: calTime = 32.0;
            break;
            case _64s: calTime = 64.0;
            break;

            default: calTime = 4.0;
            break;
        }
        return calTime;
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
        return adis.getAngle() - ((timer.get() - getCalibrationTime()) * Config.IMUDriftConstant);
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
