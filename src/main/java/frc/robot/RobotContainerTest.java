package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DriveManual;
import frc.robot.subsystems.*;

/*
* A class that contains all the subsystems and commands that Robot needs. Based off of the RobotContainer
* example class by WPI.
*
*/
public class RobotContainerTest
{

    // --------------------------------------------------------------------------------------------
    // -- Subsystems
    private final ArmSubsystem arm = new ArmSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final CtrlPanelSubsystem turner = new CtrlPanelSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem();
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();
    private final PowerDistributionPanel pdp = new PowerDistributionPanel();

    // -- Input
    private final OperatorInterface OI = new OperatorInterface();

    // -- Auto
    SendableChooser<Command> Chooser = new SendableChooser<>();


    // --------------------------------------------------------------------------------------------
    public RobotContainerTest()
    {
        // -- Bindings
        ConfigureButtonBindings_Driver();
        ConfigureButtonBindings_Operator();

        SetDefaultCommands();
    }

    // --------------------------------------------------------------------------------------------
    private void SetDefaultCommands()
    {
        if (Config.isDriveInstalled) {
            drive.setDefaultCommand(new DriveManual(OI.DriverLeft, OI.DriverRight, drive));
        }
    }

    // --------------------------------------------------------------------------------------------
    private void ConfigureButtonBindings_Driver()
    {

    }

    // --------------------------------------------------------------------------------------------
    private void ConfigureButtonBindings_Operator()
    {
        int maxShooterIntakeSpeed = 1;

        // Button A (Intake while held)
        new JoystickButton(OI.OperatorController, XboxController.Button.kA.value)
                .whileHeld(new StartEndCommand(
                        () -> {
                            intake.SetPowerRaw(1);
                            shooter.SetPowerRaw(maxShooterIntakeSpeed, maxShooterIntakeSpeed);
                        },
                        () -> {
                            intake.Stop();
                            shooter.SetPowerRaw(0,0);
                        },
                        intake, shooter
                ));

        // Button B (Outtake while held)
        new JoystickButton(OI.OperatorController, XboxController.Button.kB.value)
                .whileHeld( new StartEndCommand(
                        () ->
                        {
                            intake.SetPowerRaw(1);
                            shooter.SetPowerRaw(-maxShooterIntakeSpeed, -maxShooterIntakeSpeed);
                        },
                        () ->
                        {
                            intake.Stop();
                            shooter.SetPowerRaw(0,0);
                        },
                        intake, shooter
                ));
    }

    // --------------------------------------------------------------------------------------------
    public Command GetAutonomousCommand()
    {
        return null;
    }

}
