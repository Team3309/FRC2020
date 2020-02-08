package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.select.*;
import frc.robot.commands.drive.DriveManual;
import frc.robot.commands.drive.DriveSimpleTest;
import frc.robot.commands.shooter.FireManual;
import frc.robot.commands.shooter.StopFlywheel;
import frc.robot.subsystems.*;

/*
* A class that contains all the subsystems and commands that Robot needs. Based off of the RobotContainer
* example class by WPI.
*
*/
public class RobotContainer
{
    public static void setPowerCellHandlingState(PowerCellHandlingState newState) {
        RobotContainer.state = newState;
    }

    public static PowerCellHandlingState getPowerCellHandlingState() {
        return state;
    }

    public static enum PowerCellHandlingState {
        ARM_UP_DRIVE, SCAN, SINGLE_SHOT, MULTI_SHOT, TRENCH_DRIVE, INTAKE, READY_TO_SHOOT,
        INITIALIZE_ARM_UP_DRIVE, INIT_SCAN, INIT_SINGLE_SHOT, INIT_MULTI_SHOT, INIT_TRENCH_DRIVE, INIT_INTAKE,
        INIT_READY_TO_SHOOT
    }
    private static PowerCellHandlingState state = PowerCellHandlingState.ARM_UP_DRIVE;
    // --------------------------------------------------------------------------------------------
    // -- Subsystems
    private final ArmSubsystem arm = new ArmSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final CtrlPanelSubsystem turner = new CtrlPanelSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem(this);
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem(this);
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(this);
    private final PowerDistributionPanel pdp = new PowerDistributionPanel();

    // -- Input
    private final OperatorInterface OI = new OperatorInterface();

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
        // Testing
        // should these be saved and stored in a variable? -Tim Kavner
        new JoystickButton(OI.OperatorController, XboxController.Button.kA.value)
                .whileHeld(new DriveSimpleTest(0.1, drive));

        new JoystickButton(OI.OperatorController, XboxController.Button.kB.value)
                .whileHeld(new RunCommand(() -> drive.SetLeftRight(ControlMode.PercentOutput, 0.2, 0.2), drive)
                        .beforeStarting(() -> System.out.println("before start B")));

        new JoystickButton(OI.OperatorController, XboxController.Button.kX.value)
                .whileHeld(new RunCommand(() -> new FireManual(shooter, OI.OperatorController)));

        new JoystickButton(OI.OperatorController, XboxController.Button.kBumperRight.value)
                .whenPressed(new RunCommand(() -> new SelectReadyToShootToDriving(intake, indexer, shooter, arm)));

        new JoystickButton(OI.OperatorController, XboxController.Axis.kLeftTrigger.value)
                .whenPressed(new RunCommand(() -> new SelectIntakeToggle(intake, indexer, shooter, arm)
                ));

        new JoystickButton(OI.OperatorController, XboxController.Axis.kRightTrigger.value)
                .whenPressed(new RunCommand(() -> new SelectScan(intake, indexer, shooter)
                ));

        new JoystickButton(OI.OperatorController, XboxController.Button.kBumperLeft.value)
                .whenPressed(new RunCommand(() -> new SelectMultishot(intake, indexer, shooter)
                ));

        //D East / Right TODO are these angles correct?
        new POVButton(OI.OperatorController, 0, OI.OperatorController.getPOV())
                .whenPressed(new RunCommand(() -> new SelectReadyToShoot(ArmSubsystem.ArmPosition.longRange, intake, indexer, shooter)
                ));
        //D North / Up
        new POVButton(OI.OperatorController, 90, OI.OperatorController.getPOV())
                .whenPressed(new RunCommand(() -> new SelectReadyToShoot(ArmSubsystem.ArmPosition.midRange, intake, indexer, shooter)
                ));
        //D West / Left
        new POVButton(OI.OperatorController, 180, OI.OperatorController.getPOV())
                .whenPressed(new RunCommand(() -> new SelectReadyToShoot(ArmSubsystem.ArmPosition.closeRange, intake, indexer, shooter)
                ));

        //D South / Down
        new POVButton(OI.OperatorController, 270, OI.OperatorController.getPOV())
                .whenPressed(new RunCommand(() -> new SelectIntakeToTrench(intake, indexer, shooter, arm)
                ));

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

    public double GetCurrent(int channel) {
        return pdp.getCurrent(channel);
    }

    private void OutputToDashboard() {
        drive.OutputToDashboard();
    }

}
