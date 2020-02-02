package org.usfirst.frc.team3309;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc.team3309.subsystems.*;

/*
* A class that contains all the subsystems and commands that Robot needs. Based off of the RobotContainer
* example class by WPI.
*
*/
public class RobotContainer {

    //Constructs instances of our subsystems.
    private final ClimberSubsystem Climber = new ClimberSubsystem();
    private final CtrlPanelSubsystem Turner = new CtrlPanelSubsystem();
    private final DriveSubsystem Drive = new DriveSubsystem();
    private final IndexerSubsystem Indexer = new IndexerSubsystem();
    private final IntakeSubsystem Intake = new IntakeSubsystem();
    private final ShooterSubsystem Shooter = new ShooterSubsystem();
    private final VisionSubsystem Vision = new VisionSubsystem();

    SendableChooser<Command> chooser = new SendableChooser<>();
    public RobotContainer() {

        ConfigureButtonBindings();

    }

    private void ConfigureButtonBindings() {



    }

    public Command GetAutonomousCommand() {
        return chooser.getSelected();
    }
}
