package org.usfirst.frc.team3309.commands.pcintake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Constants;
import org.usfirst.frc.team3309.Robot;

public class IntakeForward extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.intake)) return;
        Robot.intake.actuate(Constants.INTAKE_MOTOR_STANDARD_VELOCITY);
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
