package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

import javax.security.auth.login.Configuration;

public class ArmSubsystem extends SubsystemBase {


    public enum ArmPosition {
        max(0),
        longRange(1),
        midRange(2),
        closeRange(3),
        trench(4),
        min(5),
        intermediate(6);

        int value;

        ArmPosition(int value){this.value = value;}
    }

    private WPI_TalonFX armMotor;


    public ArmSubsystem() {
        if (Config.isArmInstalled) {
            armMotor = new WPI_TalonFX(Config.ArmMotorId);
        }
    }

    public void MoveArmManually(double position) {
        if (Config.isArmInstalled) {
            armMotor.set(ControlMode.Position, position);
        }
    }

    public void MoveToPosition(ArmPosition position) {

        if (position == ArmPosition.max) {
            MoveArmManually(armMotor.getSelectedSensorPosition() - Config.ArmMaxAnglePosition);
            if (Config.isInDebug) DriverStation.reportError("Arm at max angle.", false);
        } else if (position == ArmPosition.longRange) {
            MoveArmManually(armMotor.getSelectedSensorVelocity() - Config.ArmLongRangeAnglePosition);
            if (Config.isInDebug) DriverStation.reportError("Arm at long range angle.", false);
        } else if (position == ArmPosition.midRange) {
            MoveArmManually(armMotor.getSelectedSensorPosition() - Config.ArmMidRangeAnglePosition);
            if (Config.isInDebug) DriverStation.reportError("Arm at mid range angle.", false);
        } else if (position == ArmPosition.closeRange) {
            MoveArmManually(armMotor.getSelectedSensorPosition() - Config.ArmCloseRangeAnglePosition);
            if (Config.isInDebug) DriverStation.reportError("Arm at close range angle.", false);
        } else if (position == ArmPosition.trench) {
            MoveArmManually(armMotor.getSelectedSensorPosition() - Config.ArmTrenchDriveAnglePosition);
            if (Config.isInDebug) DriverStation.reportError("Arm at trench driving angle.", false);
        } else if (position == ArmPosition.min) {
            MoveArmManually(armMotor.getSelectedSensorPosition() - Config.ArmMinAnglePosition);
            if (Config.isInDebug) DriverStation.reportError("Arm at min angle.", false);
        }
    }
}
