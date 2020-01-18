package org.usfirst.frc.team3309.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.subsystems.Vision;

public class IlluminationOn extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.vision);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.vision)) return;
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
