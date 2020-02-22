package frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Util3309;
import frc.robot.util.Waypoint;

public class DriveAuto extends CommandBase {

    private enum superState {
        stopped("Stopped."),
        drivingStraight("Driving Straight."),
        spinTurning("Spin Turning."),
        mobileTurning("Mobile Turning.");

        String name;
        superState(String name) {this.name = name;}
    }

    private enum travelState {
        stopped("Stopped."),
        accelerating("Accelerating."), //accelerating to cruise speed
        cruising("Cruising."), //Moving at a set speed
        decelerating("Decelerating."); //decelerating to approach desired point

        String name;
        travelState(String name) {this.name = name;}
    }

    private enum spinTurnState {
        notStarted("Not Started."),
        accelerating("Accelerating."), //accelerating to angular cruise speed
        cruising("Cruising."), //angular cruising speed
        decelerating("Decelerating."), //decelerating to approach tweak speed
        tweaking("Tweaking."); //speed at which final heading is corrected

        String name;
        spinTurnState(String name) {this.name = name;}
    }

    double speed = 0;
    private double lastVelocity;
    Timer ControlTimer = new Timer();

    private superState superStateMachine = superState.spinTurning;
    private travelState state = travelState.stopped;
    private spinTurnState turnState = spinTurnState.notStarted;
    double encoderZeroValue;

    public static Waypoint[] testPath = {
            new Waypoint(0, 0, 0, false),
            new Waypoint(36, 36, 0, false)
    };

    final double kTurnCorrectionConstant = 0.1;
    final double kDecelerationConstant = 0.1;

    private boolean done = false;
    private Waypoint[] path;
    private int nextWaypointIndex = 0;
    private boolean endRollout;
    private double[] transformationVector = new double[2];
    private Waypoint[] workingPath = new Waypoint[2];
    private DriveSubsystem drive;
    // for autonomous path following
    public DriveAuto(Waypoint[] path, boolean endRollOut, DriveSubsystem drive) {
        this.drive = drive;
        this.path = path;
        this.endRollout = endRollOut;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        super.initialize();
        ControlTimer.reset();
        ControlTimer.start();
        System.out.println("initialized");
    }

    @Override
    public void execute() {
        if (done)
            return;

        Waypoint currentPoint = path[nextWaypointIndex];
        Waypoint nextPoint = path[nextWaypointIndex + 1];

        //transforms nextPoint so that the code operates from the correct frame of reference.
        workingPath[0] = new Waypoint(currentPoint.xFieldInches-transformationVector[0],
                currentPoint.downFieldInches - transformationVector[1],
                currentPoint.turnRadiusInches,
                currentPoint.reverse);
        workingPath[1] = new Waypoint(nextPoint.xFieldInches - transformationVector[0],
                nextPoint.downFieldInches - transformationVector[1], nextPoint.turnRadiusInches,
                nextPoint.reverse);
        double headingToNextPoint = Util3309.getHeadingToPoint(workingPath[0], workingPath[1]);
        double error = Util3309.getHeadingError(headingToNextPoint, drive);

        double inchesBetweenWaypoints =
                Util3309.distanceFormula(currentPoint.xFieldInches, currentPoint.downFieldInches,
                        nextPoint.xFieldInches, nextPoint.downFieldInches);

        transformationVector[0] = nextPoint.xFieldInches - currentPoint.xFieldInches;
        transformationVector[1] = nextPoint.downFieldInches - currentPoint.downFieldInches;
        if (superStateMachine == superState.spinTurning) {


            drive.zeroImu();
            final double kTweakThreshold = 2.0;
            double timerValue = ControlTimer.get();
            double currentAngularVelocity = 0; //positive = clockwise, negative = counterclockwise
            //checks that this is the start of auto; timer should be started and robot should not have
            //been previously started
            if (turnState == spinTurnState.notStarted) {
                ControlTimer.reset();
                turnState = spinTurnState.accelerating;
            }

            if (turnState == spinTurnState.accelerating)   {
                currentAngularVelocity = nextPoint.angAccelerationInDegsPerSec2 * timerValue;
            }
            //checks whether we should start cruising; we should have finished our acceleration phase
            //and we should be approaching our cruise velocity
            if (turnState == spinTurnState.accelerating &&
                    currentAngularVelocity > nextPoint.maxAngularSpeedInDegsPerSec) {
                turnState = spinTurnState.cruising;
            }
            if (turnState == spinTurnState.cruising) {
                currentAngularVelocity = nextPoint.maxAngularSpeedInDegsPerSec;
            }
            //checks whether we should start decelerating; we should have completed cruising phase
            if (timerValue * nextPoint.maxAngularSpeedInDegsPerSec > error) {
                turnState = spinTurnState.decelerating;
                //separate timer to help us decelerate down from a fixed velocity
                lastVelocity = DriveSubsystem.encoderVelocityToDegsPerSec(drive.getLeftEncoderVelocity() +
                        drive.getRightEncoderVelocity())/2;
                ControlTimer.reset();
                timerValue = 0;
            }

            //
            if (turnState == spinTurnState.decelerating) {

                currentAngularVelocity = lastVelocity - (nextPoint.angDecelerationInDegsPerSec2 * timerValue);

            }
            //checks that we have completed deceleration phase and are approaching our tweaking speed
            if (turnState == spinTurnState.decelerating && currentAngularVelocity < nextPoint.angCreepSpeedInDegsPerSec) {
                turnState = spinTurnState.tweaking;
            }
            if (turnState == spinTurnState.tweaking) {
                //check if correction is needed
                if (Math.abs(error) < kTweakThreshold) {
                    //spin Turn complete
                    drive.setLeftRight(ControlMode.PercentOutput, 0, 0);
                    superStateMachine = superState.drivingStraight;
                    turnState = spinTurnState.notStarted;
                }
                //turn right if we undershot

                else if (Util3309.getHeadingError(headingToNextPoint, drive) < 0) {
                    currentAngularVelocity = nextPoint.angCreepSpeedInDegsPerSec;
                }
                //turn left if we overshot
                else if (Util3309.getHeadingError(headingToNextPoint, drive) > 0) {
                    currentAngularVelocity = -nextPoint.angCreepSpeedInDegsPerSec;
                    DriverStation.reportError("Overshot.", false);
                }
            }

            double right = drive.degreesPerSecToEncoderVelocity(currentAngularVelocity);
            drive.setLeftRight(ControlMode.Velocity, right, -right);


            if (RobotContainer.getDriveDebug()) {
                SmartDashboard.putNumber("Single-motor velocity:", currentAngularVelocity);
                SmartDashboard.putNumber("Heading error:", error);
                SmartDashboard.putString("Spin turn state:", turnState.name);
            }

        } else if (superStateMachine == superState.drivingStraight) {
            /*
             * Drive Straight
             *
             * Accelerate until the robot has reached the maximum speed specified for that
             * waypoint. Then remain at a constant speed until robot enters the deceleration
             * zone. Finally, decelerate to the slowest allowed speed until destination has
             * been reached.
             *
             *          │    ______________
             *          │   /              \
             * Velocity │  /                \
             *          │ /                  \___
             *          │/                       \
             *           -------------------------
             *                    Time
             *
             * Prepare to enter the logic jungle. You have been warned
             * JK, it actually uses some pretty simple state machine logic:
             *
             * if we are stopped, then
             *     calibrate
             *     set state to accelerating
             * if state is accelerating, then
             *     if we still need to accelerate, then
             *         accelerate
             *     else
             *         set state to cruising
             * if state is cruising, then
             *     if we still need to cruise, then
             *         cruise
             *     else
             *         set state to decelerating
             * if state is decelerating, then
             *     if we still need to decelerate, then
             *         decelerate
             *     else
             *         stop the robot and increment nextWaypointIndex
             */
            double encoderTicks = (drive.getLeftEncoderPosition() + drive.getRightEncoderPosition())/2;
            double encoderTicksTraveled = encoderTicks - encoderZeroValue;
            double inchesTraveled = DriveSubsystem.encoderCountsToInches((int) encoderTicksTraveled);

            double turnCorrection = Util3309.getHeadingError(headingToNextPoint, drive) * kTurnCorrectionConstant;

            if (state == travelState.stopped) {
                ControlTimer.reset();
                state = travelState.accelerating;
                encoderZeroValue = encoderTicks;
            }
            if (state == travelState.accelerating) {
                speed = DriveSubsystem.encoderVelocityToInchesPerSec(nextPoint.linAccelerationEncoderCtsPer100ms2 * ControlTimer.get());
                if (speed > nextPoint.maxLinSpeedEncoderCtsPer100ms) {


                    state = travelState.cruising;
                }
            }
            if (state == travelState.cruising){
                if (inchesBetweenWaypoints - inchesTraveled < speed * kDecelerationConstant) {
                    speed = nextPoint.maxLinearSpeed;
                } else {
                    state = travelState.decelerating;
                    ControlTimer.reset();
                }
            }
            if (state == travelState.decelerating){

                if (inchesTraveled < inchesBetweenWaypoints - nextPoint.linToleranceInInches) {
                    speed = nextPoint.linAccelerationEncoderCtsPer100ms2 * ControlTimer.get();
                    if (speed < DriveSubsystem.encoderVelocityToDegsPerSec(nextPoint.linCreepSpeedEncoderCtsPer100ms)) {
                        speed = nextPoint.linCreepSpeed;

                    }
                } else {
                    if (nextWaypointIndex == path.length - 1 && !endRollout) {
                        //We are done with following the path, we have arrived at the destination
                        //Stop the robot
                        speed = 0;
                    }
                    if (inchesTraveled > inchesBetweenWaypoints + nextPoint.linToleranceInInches) {
                        DriverStation.reportError("Traveled too far", true);
                    }
                    nextWaypointIndex++;
                    superStateMachine = superState.stopped;
                }
            }

            if (speed != 0 ) {
                drive.setArcade(ControlMode.Velocity, DriveSubsystem.inchesPerSecondToEncoderVelocity(speed), turnCorrection);
            } else {
                //If speed is zero, then use PercentOutput so we don't apply brakes
                drive.setArcade(ControlMode.PercentOutput, 0,0);
            }

            if (RobotContainer.getDriveDebug()) {
                SmartDashboard.putString("State:", String.valueOf(state));
                SmartDashboard.putNumber("Heading error:", Util3309.getHeadingError(headingToNextPoint, drive));
                SmartDashboard.putNumber("Throttle:", speed);

            }

            //End of Drive straight code
        } else if (superStateMachine == superState.mobileTurning) {
            //Turn on a circle:
            //Find the length of the arc defined by turnRadiusInches, and determine
            //what the velocity the robot will have based on its current velocity.
            //Assume that the arc velocity is the average of the velocity of the two wheels.
            //Find out what velocity each wheel will have to maintain to achieve this arc velocity.
            //Set each wheel's velocity to these values.
            double arcLengthInInches = nextPoint.turnRadiusInches * (180-headingToNextPoint);
            //find velocity at which the robot will travel while on the arc.

        } else if (superStateMachine == superState.stopped) {
            drive.setLeftRight(ControlMode.PercentOutput, 0, 0);
        }

        if (nextWaypointIndex == path.length - 1) {
            done = true;
        }

        if (RobotContainer.getDriveDebug()) {
            SmartDashboard.putString("Robot Super State:", superStateMachine.name);
            SmartDashboard.putString("Straight Drive State:", state.name);
            SmartDashboard.putString("Spin Turning State:", turnState.name);
            SmartDashboard.putNumber("Previous velocity:", lastVelocity);
            SmartDashboard.putNumber("Path Array Index:", nextWaypointIndex);
            DriverStation.reportWarning("Executed.", false);
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
