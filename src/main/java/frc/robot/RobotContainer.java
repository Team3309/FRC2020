package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Auto.ThreeBallAutoDriveForward;
import frc.robot.commands.arm.ManualArmAdjustment;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.DriveManual;
import frc.robot.commands.indexer.LoadIntoArm;
import frc.robot.commands.indexer.AutoIndexIn;
import frc.robot.commands.select.*;
import frc.robot.commands.vision.IlluminationOn;
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
        INTAKE,
        READY_TO_SHOOT,
        INIT_ARM_UP_DRIVE,
        INIT_SCAN,
        INIT_SINGLE_SHOT,
        INIT_MULTI_SHOT,
        INIT_INTAKE,
        INIT_READY_TO_SHOOT,
        INIT_POSITION_TURNER,
        TURNER_IN_POSITION,
        SPIN_TURNER,
        INIT_OUTTAKE,
        OUTTAKE,
        INIT_READY_TO_CLIMB,
        READY_TO_CLIMB,
        INIT_CLIMBING,
        CLIMBING
    }

    private static RobotState state = RobotState.ARM_UP_DRIVE;

    private final String armDashboardKey = "Display Arm Values";
    private final String climberDashboardKey = "Display Climber Values";
    private final String ctrlPanelDashboardKey = "Display CtrlPanel Values";
    private final static String driveDashboardKey = "Display Drive Values";
    private final static String indexerDashboardKey = "Display Indexer Values";
    private final String intakeDashboardKey = "Display Intake Values";
    private final String shooterDashboardKey = "Display Shooter Values";
    private final String visionDashboardKey = "Display Vision Values";
    private final String ArmCoastModeDashboardKey = "Arm - Coast Mode";
    private final String DriveCoastModeDashboardKey = "Drive - Coast Mode";
    private final String ClimberCoastModeDashboardKey = "Climber - Coast Mode";

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
/*
    // Intake/Outtake command groups
    private new ToOuttakeCmdGroup(intake, shooter, arm);
} else {
        return new ToDriveCmdGroup(Config.armPositionIntakeStowedTarget, intake, shooter, arm);*/

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

    // TODO BUG: left stick, right cluster is mulshot (should be left cluster)
    // TODO BUG: Right stick, right cluster is single shot (should be left stick, right cluster)

    // TODO: verify clusters have intake (right stick, right cluster)

    /** ----------------------------------------------------------------------------------------------------------------
     * Configure the bindings for the operator controller (Xbox Controller)
     */
    private void configureButtonBindings() {
        //when active is the same as when pressed
        //when inactive is the same as when released
        //whileActiveOnce is the same as when held

        Waypoint[] waypoints = {new Waypoint(0, 0, 0, false),
                new Waypoint(Math.cos(Math.toRadians(90)),
                        Math.sin(Math.toRadians(90)),
                        0,
                        false, true)};

        // TODO: Remove after spin turn testing is complete
        // TODO: Simplify the process of command selection.
        //new JoystickButton(OI.OperatorController, XboxController.Button.kA.value)
        //       .whenPressed(new DriveAuto(waypoints, false, drive));

        // -------------------------------------------------------------------------------------------------------------
        // Control Panel
        // -------------------------------------------------------------------------------------------------------------
        new JoystickButton(OI.OperatorController, XboxController.Button.kB.value)
                .whenPressed(new SelectPositionTurner(arm, intake));

        new JoystickButton(OI.OperatorController, XboxController.Button.kY.value)
                .whenPressed(new SelectSpinTurner(drive, ctrlPanel))
                .whenReleased(new SelectStopCtrlPanelSpinning(ctrlPanel, drive));

        // -------------------------------------------------------------------------------------------------------------
        // Intake / outtake
        // -------------------------------------------------------------------------------------------------------------
        new JoystickButton(OI.OperatorController, XboxController.Button.kBumperLeft.value)
                .whenPressed(new SelectToIntake(intake, indexer, shooter, arm))
                .whenReleased(new SelectCancelIntake(intake, indexer, shooter, arm, drive, ctrlPanel, OI.OperatorController));

        new JoystickButton(OI.OperatorController, XboxController.Button.kBumperRight.value)
                .whenPressed(new SelectToOuttake(intake, shooter, arm))
                .whenReleased(new SelectCancelOuttake(intake, indexer, shooter, arm, OI.OperatorController));

        // Don't require the intake subsystem for emergency outtake so it can be run in any arm up mode
        // without interrupting other commands. Arbitration over control of the intake motor is
        // managed directly by the intake subsystem.
        new JoystickButton(OI.OperatorController, XboxController.Button.kA.value)
                .whenPressed(new InstantCommand(intake::startEmergencyOuttake))
                .whenReleased(new InstantCommand(intake::stopEmergencyOuttake));

        // Manual index load
        new JoystickButton(OI.OperatorController, XboxController.Button.kX.value)
                .or(OI.rightStickRightCluster)
                .whenActive(new LoadIntoArm(indexer));

        // -------------------------------------------------------------------------------------------------------------
        // Climb
        // -------------------------------------------------------------------------------------------------------------
        new JoystickButton(OI.OperatorController, XboxController.Button.kBack.value)
                .whenPressed(new SelectClimbing(climber, intake, arm, OI.OperatorController));

        // -------------------------------------------------------------------------------------------------------------
        // Shooting
        // -------------------------------------------------------------------------------------------------------------

        // Cancel Shooting
        new POVButton(OI.OperatorController, 180)  // D-Down
                .whenPressed(new SelectReadyToShootToDriving(intake, shooter, arm));

        OI.leftStickRightCluster
                .whileActiveOnce(new SelectToSingleShot(indexer, shooter))
                .whenInactive(new SelectSingleShotToReadyToShoot(intake, indexer, shooter, arm));

        // Shoot multiple power cells in either position or velocity control mode for the indexer
        new XBoxControllerAxisButton(OI.OperatorController, XboxController.Axis.kLeftTrigger, Config.xBoxTriggerButtonThreshold)
                .or(OI.leftStickLeftCluster)
                .whenActive(new SelectToMultishot(indexer, shooter))
                .whenInactive(new SelectMultishotToReadyToShoot(intake, indexer, shooter, arm));

        //D-pad Left - Long
        new POVButton(OI.OperatorController, 270)
                .whenPressed(new SelectToReadyToShoot(Config.shooterLongRangeSolution, intake, indexer, shooter, arm)
                );

        //D-pad Up - Line
        new POVButton(OI.OperatorController, 0)
                .whenPressed(new SelectToReadyToShoot(Config.shooterMidRangeSolution, intake, indexer, shooter, arm)
                );

        //D-pad Right - Wall
        new POVButton(OI.OperatorController, 90)
                .whenPressed(new SelectToReadyToShoot(Config.shooterShortRangeSolution, intake, indexer, shooter, arm)
                );

        // Scan
        new XBoxControllerAxisButton(OI.OperatorController, XboxController.Axis.kRightTrigger, Config.xBoxTriggerButtonThreshold)
                .whenPressed(new SelectToScan(intake, indexer, shooter, arm, vision, drive))
                .whenReleased(new SelectCancelScan(intake, indexer, shooter, arm, drive, ctrlPanel));
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Push all the auto commands to the smart dashboard for easy choosing
     */
    private void setAutoOptions() {
        Chooser.addOption("Simple Auto", new ThreeBallAutoDriveForward(indexer, shooter, drive, intake, arm));
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
        arm.setCoastMode(false);

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
            arm.setCoastMode(true);
            SmartDashboard.putBoolean(ArmCoastModeDashboardKey, true);
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
        SmartDashboard.putBoolean(ClimberCoastModeDashboardKey, false);
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Reset the state of the Dashboard toggles, and their coresponding states
     * TODO: we probably want to make the default state a config value, and have a way to ask a system to reset itself
     *  so we're not duplicating logic here.
     */
    private void resetDashboardToggles() {
        // Set the arm and drive to brake mode whenever the robot is disabled.
        arm.setCoastMode(false);
        drive.setCoastMode(false);
        climber.setCoastMode(false);

        SmartDashboard.putBoolean(ArmCoastModeDashboardKey, false);
        SmartDashboard.putBoolean(DriveCoastModeDashboardKey, false);
        SmartDashboard.putBoolean(ClimberCoastModeDashboardKey, false);
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * @return the command chosen by the smartdashboard to run in auto
     */
    public Command getAutonomousCommand() {

        //return Chooser.getSelected();
        return new ThreeBallAutoDriveForward(indexer, shooter, drive, intake, arm);
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
        if (getIndexerDebug() && Config.isIndexerInstalled) {
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

        // These toggles should only be usable while disabled.
        if (DriverStation.getInstance().isDisabled()) {
            // TODO: if these are set while the robot is enabled, we should zero them back out so the dashboard doesn't
            //  show an incorrect value (again, like the other comment, we should just set these in the actual system code
            //  so they are always in the correct state.
            arm.setCoastMode(SmartDashboard.getBoolean(ArmCoastModeDashboardKey, false));
            drive.setCoastMode(SmartDashboard.getBoolean(DriveCoastModeDashboardKey, false));
            climber.setCoastMode(SmartDashboard.getBoolean(ClimberCoastModeDashboardKey, false));
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * @return boolean indicating if drive values display is enabled
     * Used for DriveAuto to output additional debug information.
     */
    public static boolean getDriveDebug() {
        return SmartDashboard.getBoolean(driveDashboardKey, false);
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * @return boolean indicating if indexer values display is enabled
     * Used for AutoIndexIn to output additional debug information.
     */
    public static boolean getIndexerDebug() {
        return SmartDashboard.getBoolean(indexerDashboardKey, false);
    }
}
