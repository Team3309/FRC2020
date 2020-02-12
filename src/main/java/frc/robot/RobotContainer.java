package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DoNothing;
import frc.robot.commands.arm.ManualArmAdjustment;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.commands.select.*;
import frc.robot.commands.drive.DriveManual;
import frc.robot.commands.drive.DriveSimpleTest;
import frc.robot.commands.shooter.FireManual;
import frc.robot.subsystems.*;

/** --------------------------------------------------------------------------------------------------------------------
* A class that contains all the subsystems and commands that Robot needs. Based off of the RobotContainer
* example class by WPI.
*/
public class RobotContainer
{

    /** ----------------------------------------------------------------------------------------------------------------
     * Robot state machine state holder.
     * TODO: The name below is probably incorrect because we're not likely to have two state machines
     *       but instead have one state machine that the entire robot shares
     * @param newState new state for the robot, this doesn't actually cause state transitions,
     *                 but instead is how state transitions mark where the robot is at
     */
    public static void setPowerCellHandlingState(PowerCellHandlingState newState) {
        RobotContainer.state = newState;
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * @return Current robot state machine state
     */
    public static PowerCellHandlingState getPowerCellHandlingState() {
        return state;
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * All states for the robot finite state machine
     */
    public enum PowerCellHandlingState {
        ARM_UP_DRIVE, SCAN, SINGLE_SHOT, MULTI_SHOT, TRENCH_DRIVE, INTAKE, READY_TO_SHOOT,
        INIT_ARM_UP_DRIVE, INIT_SCAN, INIT_SINGLE_SHOT, INIT_MULTI_SHOT, INIT_TRENCH_DRIVE, INIT_INTAKE,
        INIT_READY_TO_SHOOT
    }


    private static PowerCellHandlingState state = PowerCellHandlingState.ARM_UP_DRIVE;

    private final String armDashboardKey = "Display Arm Values";
    private final String climberDashboardKey = "Display Climber Values";
    private final String ctrlPanelDashboardKey = "Display CtrlPanel Values";
    private final static String driveDashboardKey = "Display Drive Values";
    private final String indexerDashboardKey = "Display Indexer Values";
    private final String intakeDashboardKey = "Display Intake Values";
    private final String shooterDashboardKey = "Display Shooter Values";
    private final String visionDashboardKey = "Display Vision Values";

    // --------------------------------------------------------------------------------------------
    // -- Subsystems
    private final ArmSubsystem arm = new ArmSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final CtrlPanelSubsystem ctrlPanel = new CtrlPanelSubsystem();
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
    public RobotContainer() {
        // -- Bindings
        configureButtonBindings_Driver();
        configureButtonBindings_Operator();

        setDefaultCommands();

        setAutoOptions();

        displaySubsystemToggles();
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Set up default commands for any subsystem that needs one
     */
    private void setDefaultCommands() {
        if (Config.isDriveInstalled) {
            drive.setDefaultCommand(new DriveManual(OI.DriverLeft, OI.DriverRight, drive));
            arm.setDefaultCommand(new ManualArmAdjustment(arm, OI.OperatorController));
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Initialize smart dashboard with toggles to show and hide subsystem outputs
     */
    private void displaySubsystemToggles() {
        SmartDashboard.putBoolean(armDashboardKey, false);
        SmartDashboard.putBoolean(climberDashboardKey, false);
        SmartDashboard.putBoolean(ctrlPanelDashboardKey, false);
        SmartDashboard.putBoolean(driveDashboardKey, false);
        SmartDashboard.putBoolean(indexerDashboardKey, false);
        SmartDashboard.putBoolean(intakeDashboardKey, false);
        SmartDashboard.putBoolean(shooterDashboardKey, false);
        SmartDashboard.putBoolean(visionDashboardKey, false);
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Configure the bindings for the Driver controllers (Dual flight sticks)
     */
    private void configureButtonBindings_Driver() {

    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Configure the bindings for the operator controller (Xbox Controller)
     */
    private void configureButtonBindings_Operator() {
        // Testing
        // should these be saved and stored in a variable? -Tim Kavner
        new JoystickButton(OI.OperatorController, XboxController.Button.kA.value)
                .whenPressed(new RunCommand(() -> new SelectIntakeToOuttake(intake)))
                .whenReleased(new RunCommand(() -> new SelectOuttakeToIntake(intake)));

        //TODO why is / was this here?
        new JoystickButton(OI.OperatorController, XboxController.Button.kB.value)
                .whileHeld(new RunCommand(() -> new DoNothing())
                        .beforeStarting(() -> System.out.println("before start B")));

        new JoystickButton(OI.OperatorController, XboxController.Button.kBumperRight.value)
                .whenPressed(new RunCommand(() -> new SelectReadyToShootToDriving(intake, indexer, shooter, arm)));

        new JoystickButton(OI.OperatorController, XboxController.Button.kBumperLeft.value)
                .whenPressed(new RunCommand(() -> new SelectMultishot(intake, indexer, shooter, arm)
                ));


        new JoystickButton(OI.OperatorController, XboxController.Axis.kLeftTrigger.value)
                .whenPressed(new RunCommand(() -> new SelectIntakeToggle(intake, indexer, shooter, arm)
                ))
                .whenReleased(new RunCommand(() -> new ToDriveCommandGroup(ArmSubsystem.ArmPosition.intermediate, intake, indexer, shooter, arm)
                ));

        new JoystickButton(OI.OperatorController, XboxController.Axis.kRightTrigger.value)
                .whenPressed(new RunCommand(() -> new SelectScan(intake, indexer, shooter)
                ));



        new POVButton(OI.OperatorController, 0)
                .whenPressed(new RunCommand(() -> new SelectReadyToShoot(ArmSubsystem.ArmPosition.longRange, Double.valueOf(1.0), Double.valueOf(1.0), intake, indexer, shooter, arm)
                ));
        //D North / Up
        new POVButton(OI.OperatorController, 90)
                .whenPressed(new RunCommand(() -> new SelectReadyToShoot(ArmSubsystem.ArmPosition.midRange, Double.valueOf(0.7), Double.valueOf(0.7), intake, indexer, shooter, arm)
                ));
        //D West / Left
        new POVButton(OI.OperatorController, 180)
                .whenPressed(new RunCommand(() -> new SelectReadyToShoot(ArmSubsystem.ArmPosition.closeRange, Double.valueOf(0.5), Double.valueOf(0.5), intake, indexer, shooter, arm)
                ));

        //D South / Down
        new POVButton(OI.OperatorController, 270, OI.OperatorController.getPOV())
                .whenPressed(new RunCommand(() -> new SelectIntakeToTrench(intake, indexer, shooter, arm)
                ));

    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Push all the auto commands to the smart dashboard for easy choosing
     */
    private void setAutoOptions() {
        //Chooser.addOption("Simple Auto", new SimpleAutoCommand());
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * @return the command chosen by the smartdashboard to run in auto
     */
    public Command getAutonomousCommand() {
        return Chooser.getSelected();
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Send debug values to SmartDashboard
     */
    public void outputToDashboard() {
        if (SmartDashboard.getBoolean(armDashboardKey, false) && Config.isArmInstalled) {
            arm.outputToDashboard();
        }
        if (SmartDashboard.getBoolean(climberDashboardKey, false) && Config.isClimberInstalled) {
            climber.outputToDashboard();
        }
        if (SmartDashboard.getBoolean(ctrlPanelDashboardKey, false) && Config.isCtrlPanelInstalled) {
            ctrlPanel.outputToDashboard();
        }
        if (getDriveDebug() && Config.isDriveInstalled) {
            drive.outputToDashboard();
        }
        if (SmartDashboard.getBoolean(indexerDashboardKey, false) && Config.isIndexerInstalled) {
            indexer.outputToDashboard();
        }
        if (SmartDashboard.getBoolean(intakeDashboardKey, false) && Config.isIntakeInstalled) {
            intake.outputToDashboard();
        }
        if (SmartDashboard.getBoolean(shooterDashboardKey, false) && Config.isShooterInstalled) {
            shooter.outputToDashboard();
        }
        if (SmartDashboard.getBoolean(visionDashboardKey, false) && Config.isVisionInstalled) {
            vision.outputToDashboard();
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * @return boolean indicating if drive values display is enabled
     * Used for DriveAuto to output additional debug information.
     */
    public static boolean getDriveDebug() {
        return SmartDashboard.getBoolean(driveDashboardKey, false);
    }

}
