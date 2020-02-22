package frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveApplyPower extends InstantCommand {

    DriveSubsystem drive;
    double throttle;

    public DriveApplyPower(double throttle, DriveSubsystem drive) {
        this.drive = drive;
        this.throttle = throttle;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setArcade(ControlMode.PercentOutput, throttle, 0);
    }
}
