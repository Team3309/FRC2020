package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

public class IlluminationOff extends InstantCommand {

    VisionSubsystem vision;

    IlluminationOff (VisionSubsystem vision) {
        this.vision = vision;
    }

    @Override
    public void execute() {
        vision.setIllumination(false);
    }
}
