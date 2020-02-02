package org.usfirst.frc.team3309.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.util.CheesyDriveHelper;

public class DriveManual extends CommandBase {


    CheesyDriveHelper cheesyDrive = new CheesyDriveHelper();

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {


        double throttle = Robot.oi.leftStick.getX();
        double turn = Robot.oi.rightStick.getY();
        boolean quickturn = Robot.oi.rightStick.getTrigger();

    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
