package frc.robot;


import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DisplayWarnings;


/*
 * This is the Robot class.
 * It handles the setup for the rest of the robot code.
 */

public class Robot extends TimedRobot {

    private RobotContainer container;

    // Probably to use for a couple weeks while we finalize the real robotContainer, to use for testing hardware
    private RobotContainerTest containerTest;

    private Command autonomousCommand;

    private DisplayWarnings displayWarnings = new DisplayWarnings();

    public static final PowerDistributionPanel pdp = new PowerDistributionPanel();


    /** ----------------------------------------------------------------------------------------------------------------
     *
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

        if (Config.isTestMode) {
            containerTest = new RobotContainerTest();
        } else {
            container = new RobotContainer();
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * This function is called when the Robot enters disabled.
     * It should be used to shut down processes that should only run when the bot is sendIsEnabled.
     */
    @Override
    public void disabledInit() {
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * This function is called when the Robot enters autonomous.
     * It should be used to get the current auto mode, and launch the appropriate autonomous mode.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = container.GetAutonomousCommand();

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
        if (Config.isTestMode) {
            containerTest.outputToDashboard();
        } else {
            container.outputToDashboard();
        }
        displayWarnings.execute();
    }

}
