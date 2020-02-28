package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.arm.ManualArmAdjustment;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.DriveManual;
import frc.robot.commands.indexer.LoadIntoArm;
import frc.robot.commands.indexer.AutoIndexIn;
import frc.robot.commands.select.SelectCancelIntake;
import frc.robot.commands.select.SelectCancelOuttake;
import frc.robot.commands.select.SelectCancelScan;
import frc.robot.commands.select.SelectIntakeToOuttake;
import frc.robot.commands.select.SelectMultishotToReadyToShoot;
import frc.robot.commands.select.SelectOuttakeToIntake;
import frc.robot.commands.select.SelectPositionTurner;
import frc.robot.commands.select.SelectReadyToShootToDriving;
import frc.robot.commands.select.SelectSingleShotToReadyToShoot;
import frc.robot.commands.select.SelectSpinTurner;
import frc.robot.commands.select.SelectStopCtrlPanelSpinning;
import frc.robot.commands.select.SelectToIntake;
import frc.robot.commands.select.SelectToMultishot;
import frc.robot.commands.select.SelectToReadyToShoot;
import frc.robot.commands.select.SelectToScan;
import frc.robot.commands.select.SelectToSingleShot;
import frc.robot.commands.select.SelectToTrench;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Waypoint;
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
        ARM_UP_DRIVE,
        SCAN,
        SINGLE_SHOT,
        MULTI_SHOT,
        TRENCH_DRIVE,
        INTAKE,
        READY_TO_SHOOT,
        INIT_ARM_UP_DRIVE,
        INIT_SCAN,
        INIT_SINGLE_SHOT,
        INIT_MULTI_SHOT,
        INIT_TRENCH_DRIVE,
        INIT_INTAKE,
        INIT_READY_TO_SHOOT,
        INIT_POSITION_TURNER,
        TURNER_IN_POSITION,
        SPIN_TURNER,
        INIT_OUTTAKE,
        OUTTAKE,
        INIT_READY_TO_CLIMB,
        READY_TO_CLIMB,
        CLIMBING,
        CLIMBED
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
    private final String ArmSetManualCalibrationDashboardKey = "Arm - Set Manual Calibration Now";
    private final String ArmCoastModeDashboardKey = "Arm - Coast Mode";
    private final String DriveCoastModeDashboardKey = "Drive - Coast Mode";
    private final String VisionEnableLEDsDashboardKey = "Vision - Enable LEDs";

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
            indexer.setDefaultCommand(new AutoIndexIn(indexer, shooter));
        }
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
        Waypoint[] waypoints = {new Waypoint(0, 0, 0, false),
                new Waypoint(Math.cos(Math.toRadians(90)),
                        Math.sin(Math.toRadians(90)),
                        0,
                        false, true)};
        // TODO: Remove this after the indexer sensor is installed
        new JoystickButton(OI.OperatorController, XboxController.Button.kA.value)
                .whenPressed(new DriveAuto(waypoints, false, drive));


        // TODO: enable once indexer sensor is installed
        new JoystickButton(OI.OperatorController, XboxController.Button.kA.value)
                .or(OI.rightStickRightCluster)
                // TODO BUG: hold one trigger a, add trigger b, release b, intake is canceled instead of going back to a
                .whenActive(new SelectIntakeToOuttake(intake, shooter))
                .whenInactive(new SelectOuttakeToIntake(intake, indexer, shooter, arm));

                new JoystickButton(OI.OperatorController, XboxController.Button.kX.value)
                .or(OI.rightStickRightCluster)
                .whenActive(new LoadIntoArm(indexer));

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
                ).whenReleased(new SelectCancelIntake(intake, indexer, shooter, arm, drive, ctrlPanel, OI.OperatorController)
                );
        new XBoxControllerAxisButton(OI.OperatorController, XboxController.Axis.kRightTrigger, Config.xBoxTriggerButtonThreshold)
                .whenPressed(new SelectToTrench(intake, indexer, shooter, arm, drive, ctrlPanel))
                .whenReleased(new SelectCancelOuttake(intake, indexer, shooter, arm, drive, ctrlPanel, OI.OperatorController));

        //D-pad Left
        new POVButton(OI.OperatorController, 270)
                .whenPressed(new SelectToReadyToShoot(Config.shooterLongRangeSolution, intake, indexer, shooter, arm)
                );

         //Testing new simpler logic without firing solutions.
        //new POVButton(OI.OperatorController, 270)
                //.whenPressed(new MoveArmAndRetractIntake(-25000, intake, arm));

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
                .whenPressed(new SelectToScan(intake, indexer, shooter, arm, vision, drive))
                .whenReleased(new SelectCancelScan(intake, indexer, shooter, arm, drive, ctrlPanel));

        new JoystickButton(OI.OperatorController, XboxController.Button.kBack.value)
                .whenPressed(new DriveAuto(DriveAuto.testPath, false, drive));
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Push all the auto commands to the smart dashboard for easy choosing
     */
    private void setAutoOptions() {
        //Chooser.addOption("Simple Auto", new SimpleAutoCommand());
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Passthrough of robotPeriodic
     */
    public void robotPeriodic() {
        outputToDashboard();
        updateDashboardToggles();
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Called once when the robot enables.
     */
    public void onEnabled() {
        resetDashboardToggles();
    }


    /** ----------------------------------------------------------------------------------------------------------------
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

        resetDashboardToggles();
    }


    /** ----------------------------------------------------------------------------------------------------------------
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

        // Toggles for systems while disabled.
        SmartDashboard.putBoolean(ArmCoastModeDashboardKey, false);
        SmartDashboard.putBoolean(DriveCoastModeDashboardKey, false);
        SmartDashboard.putBoolean(VisionEnableLEDsDashboardKey, true);

        // Technically not a display toggle, but the button that lets you manually calibrate
        //SmartDashboard.putBoolean(ArmSetManualCalibrationDashboardKey, false);
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Reset the state of the Dashboard toggles, and their coresponding states
     * TODO: we probably want to make the default state a config value, and have a way to ask a system to reset itself
     *  so we're not duplicating logic here.
     */
    private void resetDashboardToggles() {
        // Set the arm and drive to brake mode whenever the robot is disabled.
        arm.setBrakeMode();
        drive.setCoastMode(false);

        SmartDashboard.putBoolean(ArmCoastModeDashboardKey, false);
        SmartDashboard.putBoolean(DriveCoastModeDashboardKey, false);
        SmartDashboard.putBoolean(VisionEnableLEDsDashboardKey, true);
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
    private void outputToDashboard() {

        // TODO: Jleyshock - delete values when a bool is unchecked

        SmartDashboard.putString("PC Handling State", state.name());
        if (Config.isArmInstalled) {
            if (SmartDashboard.getBoolean(armDashboardKey, false)) {
                arm.outputToDashboard();
            } else {
                arm.outputArmPositionToDashboard();
            }
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
     * Read values that are toggleable on the dashboard and update state as appropriate
     */
    private void updateDashboardToggles() {
        // Manual calibration is currently disabled, instead we calibrate via turning no the robot in the correct state
//        boolean setCalibration = SmartDashboard.getBoolean(ArmSetManualCalibrationDashboardKey, false);
//        if (setCalibration) {
//            SmartDashboard.putBoolean(ArmSetManualCalibrationDashboardKey, false);
//            arm.calibrate();
//        }

        // These toggles should only be usable while disabled.
        if (DriverStation.getInstance().isDisabled()) {
            if (SmartDashboard.getBoolean(ArmCoastModeDashboardKey, false)) {
                arm.setCoastMode();
            } else {
                arm.setBrakeMode();
            }

            drive.setCoastMode(SmartDashboard.getBoolean(DriveCoastModeDashboardKey, false));
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
