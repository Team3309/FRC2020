package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DriveManual;
import frc.robot.subsystems.*;

/** --------------------------------------------------------------------------------------------------------------------
* Temp class that replaces RobotContainer that we can use to test early hardware that either is incompatible
 * with the real code because hardware is missing, or because real code isn't in a working state and we need
 * to actuate something on the hardware.
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

    // -- Input
    private final OperatorInterface OI = new OperatorInterface();

    // -- Auto
    SendableChooser<Command> Chooser = new SendableChooser<>();


    /** ----------------------------------------------------------------------------------------------------------------
     * Constructor
     */
    public RobotContainerTest() {
        // -- Bindings
        ConfigureButtonBindings_Driver();
        ConfigureButtonBindings_Operator();

        setDefaultCommands();
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Set up default commands for any subsystem that needs one
     */
    private void setDefaultCommands() {
        if (Config.isDriveInstalled) {
            drive.setDefaultCommand(new DriveManual(OI.DriverLeft, OI.DriverRight, drive));
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Configure the bindings for the Driver controllers (Dual flight sticks)
     */
    private void ConfigureButtonBindings_Driver() {

    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Configure the bindings for the operator controller (Xbox Controller)
     */
    private void ConfigureButtonBindings_Operator() {
        double maxShooterIntakePower = 0.4;

        // Button A (Intake while held)
        new JoystickButton(OI.OperatorController, XboxController.Button.kA.value)
                .whileHeld(new StartEndCommand(
                        () -> {
                            intake.intake();
                            shooter.setPowerRaw(maxShooterIntakePower, maxShooterIntakePower);
                        },
                        () -> {
                            intake.stop();
                            shooter.stopFlywheels();
                        },
                        intake, shooter
                ));

        // Button B (Outtake while held)
        new JoystickButton(OI.OperatorController, XboxController.Button.kB.value)
                .whileHeld(new StartEndCommand(
                        () -> {
                            intake.outtake();
                            shooter.stopFlywheels();
                        },
                        () -> {
                            intake.stop();
                            shooter.stopFlywheels();
                        },
                        intake, shooter
                ));
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Send debug values to SmartDashboard
     */
    public void outputToDashboard() { }

}
