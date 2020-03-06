package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

public class IlluminationOn extends InstantCommand {

    VisionSubsystem vision;

    public IlluminationOn (VisionSubsystem vision) {
        this.vision = vision;
    }

    @Override
    public void execute() {
        //vision.setIllumination(true);
    }
}