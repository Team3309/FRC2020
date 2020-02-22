package frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveApplyPower extends CommandBase {

    DriveSubsystem drive;
    double throttle;

    public DriveApplyPower(double throttle, DriveSubsystem drive) {
        this.drive = drive;
        this.throttle = throttle;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drive.setArcade(ControlMode.PercentOutput, throttle, 0);
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
