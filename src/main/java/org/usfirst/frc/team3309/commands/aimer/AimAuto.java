package org.usfirst.frc.team3309.commands.aimer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Constants;
import org.usfirst.frc.team3309.Robot;

public class AimAuto extends CommandBase {

    PIDController turnPID;

    @Override
    public void initialize() {
        turnPID = new PIDController(Constants.kAimingP, Constants.kAimingI, Constants.kAimingD);


    }

    @Override
    public void execute() {


        double turn = 0;

    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
