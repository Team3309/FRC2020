package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FiringSolution;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.groups.ToReadyToShootCommandGroup;
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

    public CreateFiringSolution(VisionSubsystem vision, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm, DriveSubsystem drive) {
        this.vision = vision;
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        this.arm = arm;
        this.drive = drive;
    }

    public boolean isFinished() {
        boolean hasTarget = vision.hasTarget();
        if (hasTarget) {
            FiringSolution firingSolution = new FiringSolution(
                    vision.getDistanceToTarget(),
                    vision.getAngleToTarget(),
                    vision.getHeightAngleToTarget());
            Waypoint[] waypoints = {new Waypoint(0, 0, 0, false),
                                    new Waypoint(0.01 * Math.cos(Math.toRadians(vision.getDistanceToTarget())),
                                            0.01 * Math.sin(Math.toRadians(vision.getDistanceToTarget())),
                                            0,
                                            false)};
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new DriveAuto(waypoints, false, drive),
                            new ToReadyToShootCommandGroup(
                                    firingSolution, intake, indexer, shooter, arm))
                    );

        } else {
            //System.out.println("No target found");
        }
        return hasTarget;
    }
}
