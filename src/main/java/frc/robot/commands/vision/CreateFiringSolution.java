package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FiringSolution;
import frc.robot.commands.groups.ToReadyToShootCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CreateFiringSolution extends CommandBase {
    private final VisionSubsystem vision;
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final ArmSubsystem arm;

    public CreateFiringSolution(VisionSubsystem vision, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        this.vision = vision;
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        this.arm = arm;
    }

    public boolean isFinished() {
        boolean hasTarget = vision.hasTarget();
        if (hasTarget) {
            CommandScheduler.getInstance().schedule(
                    new ToReadyToShootCommandGroup(
                            new FiringSolution(
                                    vision.getDistanceToTarget(),
                                    vision.getAngleToTarget(),
                                    vision.getHeightAngleToTarget()), intake, indexer, shooter, arm));
        }
        return hasTarget;
    }
}
