package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.arm.ManualArmAdjustment;
import frc.robot.commands.ctrlpanelturner.Rotate;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.DriveManual;
import frc.robot.commands.indexer.LoadIntoArm;
import frc.robot.commands.indexer.ManageIndexer;
import frc.robot.commands.select.*;
import frc.robot.subsystems.*;
import frc.robot.util.XBoxControllerAxisButton;

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
    public static void setRobotState(RobotState newState) {
        RobotContainer.state = newState;
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * @return Current robot state machine state
     */
    public static RobotState getRobotState() {
        return state;
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * All states for the robot finite state machine
     */
    public enum RobotState {
        ARM_UP_DRIVE, SCAN, SINGLE_SHOT, MULTI_SHOT, TRENCH_DRIVE, INTAKE, READY_TO_SHOOT,
        INIT_ARM_UP_DRIVE, INIT_SCAN, INIT_SINGLE_SHOT, INIT_MULTI_SHOT, INIT_TRENCH_DRIVE, INIT_INTAKE,
        INIT_READY_TO_SHOOT, INIT_POSITION_TURNER, TURNER_IN_POSITION, SPIN_TURNER
    }

    private static RobotState state = RobotState.ARM_UP_DRIVE;

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
        configureButtonBindings();
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
        }
        if (Config.isArmInstalled) {
            arm.setDefaultCommand(new ManualArmAdjustment(arm, OI.OperatorController));
        }
        if (Config.isIndexerInstalled) {
            indexer.setDefaultCommand(new ManageIndexer(indexer));
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
     * Configure the bindings for the operator controller (Xbox Controller)
     */
    private void configureButtonBindings() {

        //when active is the same as when pressed
        //when inactive is the same as when released
        //whileActiveOnce is the same as when held

        // TODO: Fix binding to leftStickRightCluster for Single SHot
        OI.leftStickRightCluster
                .whileActiveOnce(new SelectToSingleShot(indexer, shooter))
                .whenInactive(new SelectSingleShotToReadyToShoot(intake, indexer, shooter, arm));

        // TODO: Remove this after the indexer sensor is installed
        new JoystickButton(OI.OperatorController, XboxController.Button.kA.value)
                .whenPressed(new LoadIntoArm(indexer));


        // TODO: enable once indexer sensor is installed
        /*new JoystickButton(OI.OperatorController, XboxController.Button.kA.value)
                .or(OI.rightStickRightCluster)
                .whenActive(new SelectIntakeToOuttake(intake, shooter))
                .whenInactive(new SelectOuttakeToIntake(intake, shooter));*/

        new JoystickButton(OI.OperatorController, XboxController.Button.kB.value)
                .whenPressed(new SelectPositionTurner(arm, intake));

        new JoystickButton(OI.OperatorController, XboxController.Button.kY.value)
                .whenPressed(new SelectSpinTurner(drive, ctrlPanel))
                .whenReleased(new SelectStopCtrlPanelSpinning(ctrlPanel, drive));

        new JoystickButton(OI.OperatorController, XboxController.Button.kBumperRight.value)
                .whenPressed(new SelectReadyToShootToDriving(intake, indexer, shooter, arm, drive, ctrlPanel));

        new JoystickButton(OI.OperatorController, XboxController.Button.kBumperLeft.value)
                .or(OI.leftStickLeftCluster)
                .whenActive(new SelectToMultishot(indexer, shooter))
                .whenInactive(new SelectMultishotToReadyToShoot(intake, indexer, shooter, arm));

        new XBoxControllerAxisButton(OI.OperatorController, XboxController.Axis.kLeftTrigger, Config.xBoxTriggerButtonThreshold)
                .whenPressed(new SelectToIntake(intake, indexer, shooter, arm)
                ).whenReleased(new SelectCancelIntake(intake, indexer, shooter, arm, drive, ctrlPanel)
                );
        new XBoxControllerAxisButton(OI.OperatorController, XboxController.Axis.kRightTrigger, Config.xBoxTriggerButtonThreshold)
                .whenPressed(new SelectToScan(intake, indexer, shooter, arm, vision, drive))
                .whenReleased(new SelectCancelScan(intake, indexer, shooter, arm, drive, ctrlPanel));

        //D-pad Left
        new POVButton(OI.OperatorController, 270)
                .whenPressed(new SelectToReadyToShoot(Config.shooterLongRangeSolution, intake, indexer, shooter, arm)
                );

        //D-pad Up
        new POVButton(OI.OperatorController, 0)
                .whenPressed(new SelectToReadyToShoot(Config.shooterMidRangeSolution, intake, indexer, shooter, arm)
                );

        //D-pad Right
        new POVButton(OI.OperatorController, 90)
                .whenPressed(new SelectToReadyToShoot(Config.shooterShortRangeSolution, intake, indexer, shooter, arm)
                );

        //D-pad Down
        new POVButton(OI.OperatorController, 180)
                .whenPressed(new SelectToTrench(intake, indexer, shooter, arm, drive, ctrlPanel)
                );

        new JoystickButton(OI.OperatorController, XboxController.Button.kBack.value)
                .whenPressed(new DriveAuto(DriveAuto.testPath, false, drive));
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Push all the auto commands to the smart dashboard for easy choosing
     */
    private void setAutoOptions() {
        //Chooser.addOption("Simple Auto", new SimpleAutoCommand());
    }

    /**
     * Avoid surprises for safety when re-enabling
     */
    public void disabledInit() {
        //We're using a double solenoid now and want to retract but don't can't move the arm in a single cycle if intaking
        //so the best we can do is nothing, which is fine because the double solenoid holds it state when reenabled.
        //TODO: Add a test mode only button that lets us have the robot send itself to starting configuration.

        // let arm drop slowly
        arm.setBrakeMode();

        // Cancel previous goal position so arm doesn't snap back to where it had been when re-enabled
        arm.stopMotor();

        // Clear a pending X button press so we don't accidentally release the arm brake the instant we are disabled
        OI.OperatorController.getXButtonPressed();
    }

    /**
     * Continuously called while we are disabled
     */
    public void disabledPeriodic() {

        // Don't allow indexer to move when re-enabled if it is manually moved while disabled
        indexer.reset();

        // Allow arm to be moved when disabled
        if (OI.OperatorController.getXButtonPressed()) {
            arm.setCoastMode();
        }
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

        SmartDashboard.putString("PC Handling State", state.name());
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
