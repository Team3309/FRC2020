package frc.robot.commands.scan;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class FindTarget extends CommandBase {

    private VisionSubsystem vision;

    public FindTarget(VisionSubsystem vision) {
        this.vision = vision;
        addRequirements(vision);
    }


}
