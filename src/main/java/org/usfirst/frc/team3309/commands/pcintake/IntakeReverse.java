package org.usfirst.frc.team3309.commands.pcintake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.subsystems.PCIntake;

public class IntakeReverse extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.intake)) return;
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
