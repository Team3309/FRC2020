package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//probably make in to 3 files?
public class EngageIntakeFlywheelAndIndexer extends CommandBase {

    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;


    public EngageIntakeFlywheelAndIndexer(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter) {

        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(intake, indexer, shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.Spin(1);
        indexer.Load();
        shooter.SpinUpFlywheels();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
