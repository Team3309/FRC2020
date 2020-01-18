package org.usfirst.frc.team3309.commands.aimer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Constants;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.subsystems.Aimer;

public class AimAuto extends CommandBase {

    PIDController turnPID;

    @Override
    public void initialize() {
        turnPID = new PIDController(Constants.kAimingP, Constants.kAimingI, Constants.kAimingD);

        addRequirements(Robot.aimer);
        addRequirements(Robot.drive);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.aimer) || !hasRequirement(Robot.drive)) return;

        double turn = turnPID.calculate(Robot.vision.limelight.getTx(), 0);

        Robot.drive.setLeftRight(ControlMode.PercentOutput, -turn, turn);
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
