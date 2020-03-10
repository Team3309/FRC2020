package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FiringSolution;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Waypoint;

public class CreateFiringSolution extends CommandBase {
    private final VisionSubsystem vision;
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final ArmSubsystem arm;
    private final DriveSubsystem drive;
    private boolean hasTarget;

    public CreateFiringSolution(VisionSubsystem vision, IntakeSubsystem intake, IndexerSubsystem indexer,
                                ShooterSubsystem shooter, ArmSubsystem arm, DriveSubsystem drive) {
        this.vision = vision;
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        this.arm = arm;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        hasTarget = false;
    }

    @Override
    public void execute() {
        if (!hasTarget) {  // only engage target once
            hasTarget = vision.hasTarget();
            if (hasTarget) {
                FiringSolution firingSolution = new FiringSolution(
                        vision.getDistanceToTarget(),
                        vision.getAngleToTarget(),
                        vision.getHeightAngleToTarget());
                Waypoint[] path = {new Waypoint(0, 0, 0, false,
                        vision.getAngleToTarget() - drive.getAngularPosition())};

                CommandScheduler.getInstance().schedule(new DriveAuto(path, false, drive));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return hasTarget;
    }
}
