package org.usfirst.frc.team3309.commands.balancer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.subsystems.Balancer;

public class ManualShift extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.balancer);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.balancer)) return;
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
