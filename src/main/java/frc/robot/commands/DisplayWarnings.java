package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
        StringBuilder warnMsg = new StringBuilder("");
        conditionalMsg(!Config.isArmInstalled, warnMsg, "Arm");
        conditionalMsg(!Config.isClimberInstalled, warnMsg, "Climber");
        conditionalMsg(!Config.isCtrlPanelInstalled, warnMsg, "CtrlPanel");
        conditionalMsg(!Config.isDriveInstalled, warnMsg, "Drive");
        conditionalMsg(!Config.isIndexerInstalled, warnMsg, "Indexer");
        conditionalMsg(!Config.isIntakeInstalled, warnMsg, "Intake");
        conditionalMsg(!Config.isShooterInstalled, warnMsg, "Shooter");
        conditionalMsg(!Config.isVisionInstalled, warnMsg, "Vision");
        if (!warnMsg.toString().isEmpty()) {
            DriverStation.reportError("Not installed: " + warnMsg, false);
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

