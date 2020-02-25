package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Config;

public class DisplayWarnings {
    private Timer warningsTimer = new Timer();

    public DisplayWarnings() {
        warningsTimer.start();
        warnNow();
    }

    public void execute() {
        if (warningsTimer.get() >= 20) {
            warnNow();
            warningsTimer.reset();
        }
    }

    private void warnNow() {
        if (Config.isArmInstalled && Config.armPIDTuningMode) {
            DriverStation.reportError("DANGER: Arm is in PID tuning mode!!!!", false);
        }
        else {
            StringBuilder warnMsg = new StringBuilder();
            conditionalMsg(!Config.isCompressorEnabled, warnMsg, "Compressor");
            conditionalMsg(!Config.isPcmInstalled, warnMsg, "PCM");
            conditionalMsg(!Config.isArmInstalled, warnMsg, "Arm");
            conditionalMsg(!Config.isClimberInstalled, warnMsg, "Climber");
            conditionalMsg(!Config.isCtrlPanelInstalled, warnMsg, "CtrlPanel");
            conditionalMsg(!Config.isDriveInstalled, warnMsg, "Drive");
            conditionalMsg(!Config.isIndexerInstalled, warnMsg, "Indexer");
            conditionalMsg(!Config.isIntakeInstalled, warnMsg, "Intake");
            conditionalMsg(!Config.isShooterInstalled, warnMsg, "Shooter");
            conditionalMsg(!Config.isVisionInstalled, warnMsg, "Vision");
            conditionalMsg(!Config.isLimelightOn, warnMsg, "Limelight LED");
            conditionalMsg(!Config.isIMUInstalled, warnMsg, "IMU");
            if (!warnMsg.toString().isEmpty()) {
                DriverStation.reportWarning("Not installed: " + warnMsg, false);
            }
        }
    }

    private void conditionalMsg(boolean condition, StringBuilder msg, String condMsg) {
        if (condition) {
            if (!msg.toString().isEmpty()) {
                msg.append(", ");
            }
            msg.append(condMsg);
        }
    }
}

