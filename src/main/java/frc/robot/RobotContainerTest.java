package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.drive.DriveManual;
import frc.robot.commands.drive.DriveSimpleTest;
import frc.robot.commands.select.SelectIntakeToggle;
import frc.robot.commands.select.SelectMultishot;
import frc.robot.commands.select.SelectReadyToShoot;
import frc.robot.commands.select.SelectScan;
import frc.robot.commands.shooter.FireManual;
import frc.robot.commands.shooter.StopFlywheel;
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

    }

    // --------------------------------------------------------------------------------------------
    public Command GetAutonomousCommand()
    {
        return null;
    }

    // --------------------------------------------------------------------------------------------
    public void OutputToDashboard()
    {
        drive.OutputToDashboard();
    }

}
