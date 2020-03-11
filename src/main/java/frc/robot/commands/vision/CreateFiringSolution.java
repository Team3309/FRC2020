package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Config;
import frc.robot.FiringSolution;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.FiringSolutionManager;
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
                double rangeInches = vision.getDistanceToTarget();

                double theta = Math.toRadians(vision.getAngleToTarget()); // we rename these for sake of sanity with the math
                double phi = Math.toRadians(vision.getHeightAngleToTarget());
                double depthToTarget = rangeInches * Math.cos(phi) * Math.cos(theta);
                double lengthToTarget = rangeInches * Math.cos(phi) * Math.sin(theta);
                double heightToTarget = rangeInches * Math.sin(phi);

                //from here we can add value to depth height and length easily because we are relative to the back hole in its basis.
                double depthToThreePointGoal = depthToTarget + Config.fieldVisionDepthOfThreePointHoleFromVisionTarget;
                double lengthToThreePointGoal = lengthToTarget;
                double heightToThreePointGoal = heightToTarget + Config.fieldVisionHeightOfThreePointHoleFromVisionTarget;

                //
                double threePointHoleDistance = Math.sqrt(depthToThreePointGoal * depthToThreePointGoal + lengthToThreePointGoal * lengthToThreePointGoal + heightToThreePointGoal * heightToThreePointGoal);
                //WE'LL NEED THESE LATER HOLD ON TO THE FORMULAE FOR NOW
                double threePointHoleTx = Math.toDegrees(Math.asin(heightToThreePointGoal / threePointHoleDistance)); //aka adjusted phi, aka the angle we need to rotate by to be facing the 3 point goal

                FiringSolution firingSolution = FiringSolutionManager.getSingleton().lookupFiringSolution(threePointHoleDistance, Config.visionFiringSolutionTag);
                Waypoint[] path = {new Waypoint(0, 0, 0, false,
                        threePointHoleTx - drive.getAngularPosition())};

                CommandScheduler.getInstance().schedule(new DriveAuto(path, drive));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return hasTarget;
    }
}
