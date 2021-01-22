package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class DisplayWarnings {
    private Timer warningsTimer = new Timer();

    public DisplayWarnings() {
        warningsTimer.start();
        warnNow();
    }

    public void execute() {
        if (warningsTimer.get() >= 60) {
            warnNow();
            warningsTimer.reset();
        }
    }

    private void warnNow() {
        if (Config.armAvailable && Config.armPIDTuningMode) {
            DriverStation.reportError("DANGER: Arm is in PID tuning mode!!!!", false);
        }
        else {
            StringBuilder warnMsg = new StringBuilder();
            conditionalMsg(!Config.compressorEnabled, warnMsg, "Compressor");
            conditionalMsg(!Config.pcmAvailable, warnMsg, "PCM");
            conditionalMsg(!Config.armAvailable, warnMsg, "Arm");
            conditionalMsg(!Config.climberAvailable, warnMsg, "Climber");
            conditionalMsg(!Config.ctrlPanelAvailable, warnMsg, "CtrlPanel");
            conditionalMsg(!Config.driveAvailable, warnMsg, "Drive");
            conditionalMsg(!Config.indexerAvailable, warnMsg, "Indexer");
            conditionalMsg(!Config.intakeAvailable, warnMsg, "Intake");
            conditionalMsg(!Config.shooterAvailable, warnMsg, "Shooter");
            conditionalMsg(!Config.visionAvailable, warnMsg, "Vision");
            conditionalMsg(!Config.isLimelightOn, warnMsg, "Limelight LED");
            conditionalMsg(!Config.IMUAvailable, warnMsg, "IMU");
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

