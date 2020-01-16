package org.usfirst.frc.team3309.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.subsystems.Drive;

public class DriveManual extends CommandBase {

    CheesyDriveHelper cheesyDrive = new CheesyDriveHelper();

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double throttle = 0;
        double turn = 0;
        boolean quickturn = false;

        Robot.drive.setLeftRight(cheesyDrive.update(throttle, turn, quickturn, false));
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
