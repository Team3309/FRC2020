package org.usfirst.frc.team3309;


import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.usfirst.frc.team3309.subsystems.*;


/*
 * This is the Robot class.
 * It handles the setup for the rest of the robot code.
 */

public class Robot extends TimedRobot {

    public static Aimer aimer;
    public static Balancer balancer;
    public static Climber climber;
    public static CtrlPanelTurner ctrlPanelTurner;
    public static Drive drive;
    public static PCIndexer indexer;
    public static PCIntake intake;
    public static Vision vision;
    public static PowerDistributionPanel pdp;

    public static OI oi;


    /*
     * This function is called when the Robot program starts. use it to initialize your subsystems,
     * and to set up anything that needs to be initialized with the robot.
     */

    @Override
    public void robotInit() {

        aimer = new Aimer();
        balancer = new Balancer();
        climber = new Climber();
        ctrlPanelTurner = new CtrlPanelTurner();
        drive = new Drive();
        indexer = new PCIndexer();
        intake = new PCIntake();
        vision = new Vision();
        pdp = new PowerDistributionPanel();

        oi = new OI();

        super.robotInit();
    }

    /*
     * This function is called when the Robot enters disabled.
     * It should be used to shut down processes that should only run when the bot is sendIsEnabled.
     */
    @Override
    public void disabledInit() {
    }

    /*
     * This function is called when the Robot enters autonomous.
     * It should be used to get the current auto mode, and launch the appropriate autonomous mode.
     */
    @Override
    public void autonomousInit() {
    }

    /*
     * This function is called every 20 milliseconds while the robot is in autonomous.
     * It should be used to perform periodic tasks that need to be done while the robot is in autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /*
     * This function is called when the Robot enters teleop.
     * It should be used to shut down any autonomous code, and prepare the bot for human control.
     */
    @Override
    public void teleopInit() {
    }

    /*
     * This function is called every 2 milliseconds while the robot is in autonomous.
     * It should be used to perform periodic tasks that need to be done while the robot is in teleop.
     */
    @Override
    public void teleopPeriodic() {
    }

    /*
     * This function is called when the Robot enters test mode.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    /*
     * This function always runs, regardless of mode.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /*
     * This is the main function, which is where every java program starts.
     * All we do here is insert the code that is used to start up the rest of the robot code.
     */
    public static void main(String[] args) {
        RobotBase.startRobot(Robot::new);
    }

}
