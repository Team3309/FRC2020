package org.usfirst.frc.team3309;

import org.usfirst.frc.team3309.subsystems.*;

public class RobotContainer {

    //Constructs instances of our subsystems.
    private final ClimberSubsystem Climber = new ClimberSubsystem();
    private final CtrlPanelSubsystem Turner = new CtrlPanelSubsystem();
    private final DriveSubsystem Drive = new DriveSubsystem();
    private final IndexerSubsystem Indexer = new IndexerSubsystem();
    private final IntakeSubsystem Intake = new IntakeSubsystem();
    private final ShooterSubsystem Shooter = new ShooterSubsystem();
    private final VisionSubsystem Vision = new VisionSubsystem();

    public RobotContainer() {

        ConfigureButtonBindings();

    }

    private void ConfigureButtonBindings() {
        boolean foo = true;
    }
}
