package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DriveManual;
import frc.robot.commands.drive.DriveSimpleTest;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/*
* A class that contains all the subsystems and commands that Robot needs. Based off of the RobotContainer
* example class by WPI.
*
*/
public class RobotContainer
{

    // --------------------------------------------------------------------------------------------
    // -- Subsystems
    private final ClimberSubsystem Climber = new ClimberSubsystem();
    private final CtrlPanelSubsystem Turner = new CtrlPanelSubsystem();
    private final DriveSubsystem Drive = new DriveSubsystem();
    private final IndexerSubsystem Indexer = new IndexerSubsystem();
    private final IntakeSubsystem Intake = new IntakeSubsystem();
    private final ShooterSubsystem Shooter = new ShooterSubsystem();
    private final VisionSubsystem Vision = new VisionSubsystem();

    // -- Input
    //private final OINew OI = new OINew();

    // -- Auto
    SendableChooser<Command> Chooser = new SendableChooser<>();


    // --------------------------------------------------------------------------------------------
    public RobotContainer()
    {
        // -- Bindings
        ConfigureButtonBindings_Driver();
        ConfigureButtonBindings_Operator();

        SetDefaultCommands();

        SetAutoOptions();
    }

    // --------------------------------------------------------------------------------------------
    private void SetDefaultCommands()
    {
        Drive.setDefaultCommand(new DriveManual(Drive));
    }

    // --------------------------------------------------------------------------------------------
    private void ConfigureButtonBindings_Driver()
    {

    }

    // --------------------------------------------------------------------------------------------
    private void ConfigureButtonBindings_Operator()
    {
        // Testing
        new JoystickButton(Robot.oi.operatorController, XboxController.Button.kA.value)
                .whileHeld(new DriveSimpleTest(0.1, Drive));

        new JoystickButton(Robot.oi.operatorController, XboxController.Button.kB.value)
                .whileHeld(new RunCommand(() -> Drive.SetLeftRight(ControlMode.PercentOutput, 0.2, 0.2), Drive).beforeStarting(() -> System.out.println("before start B")));
    }

    // --------------------------------------------------------------------------------------------
    private void SetAutoOptions()
    {
        //Chooser.addOption("Simple Auto", new SimpleAutoCommand());
    }

    // --------------------------------------------------------------------------------------------
    public Command GetAutonomousCommand()
    {
        return Chooser.getSelected();
    }
}
