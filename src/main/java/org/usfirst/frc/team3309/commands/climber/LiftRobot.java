package org.usfirst.frc.team3309.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.subsystems.Climber;

public class LiftRobot extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.climber);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.climber)) return;
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
