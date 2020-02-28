package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/*
 * This is the Robot class.
 * It handles the setup for the rest of the robot code.
 */

public class Robot extends TimedRobot {

    private RobotContainer container;
    private Command autonomousCommand;
    private DisplayWarnings displayWarnings = new DisplayWarnings();
    public static final PowerDistributionPanel pdp = new PowerDistributionPanel();
    private boolean wasDisabled = true;

    /** ----------------------------------------------------------------------------------------------------------------
     * Constructor
     */
    public Robot() {
        super(0.01);
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * This function is called when the Robot program starts. use it to initialize your subsystems,
     * and to set up anything that needs to be initialized with the robot.
     */
    @Override
    public void robotInit() {

        // Check if we need to disable the compressor
        if (Config.isPcmInstalled && (!Config.isCompressorEnabled || Config.armPIDTuningMode)) {
            Compressor compressor = new Compressor();
            compressor.stop();
        }

        container = new RobotContainer();
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * This function is called when the Robot enters disabled.
     * It should be used to shut down processes that should only run when the bot is sendIsEnabled.
     */
    @Override
    public void disabledInit() {
        container.disabledInit();
        wasDisabled = true;
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * This function is called when the Robot enters autonomous.
     * It should be used to get the current auto mode, and launch the appropriate autonomous mode.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = container.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * This function is called every 20 milliseconds while the robot is in autonomous.
     * It should be used to perform periodic tasks that need to be done while the robot is in autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * This function is called when the Robot enters teleop.
     * It should be used to shut down any autonomous code, and prepare the bot for human control.
     */
    @Override
    public void teleopInit() {
        if(autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * This function is called every 2 milliseconds while the robot is in autonomous.
     * It should be used to perform periodic tasks that need to be done while the robot is in teleop.
     */
    @Override
    public void teleopPeriodic() {
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * This function is called when the Robot enters test mode.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }


    /** ----------------------------------------------------------------------------------------------------------------
     *
     */
    @Override
    public void testPeriodic() {
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * This function always runs, regardless of mode.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        container.robotPeriodic();
        displayWarnings.execute();

        if (wasDisabled && DriverStation.getInstance().isEnabled()) {
            wasDisabled = false;
            container.onEnabled();
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     *
     */
    @Override
    public void disabledPeriodic() {
        //   Make sure they are enabled when the robot is enabled (related to above todo)
        container.disabledPeriodic();
    }
}
