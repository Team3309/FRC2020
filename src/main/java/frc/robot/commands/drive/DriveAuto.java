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
        stopped("Stopped"),
        drivingStraight("Driving Straight"),
        spinTurning("Spin Turning"),
        mobileTurning("Mobile Turning");

        String name;
        superState(String name) {this.name = name;}
    }

    private enum travelState {
        stopped("Stopped"),
        accelerating("Accelerating"), //accelerating to cruise speed
        cruising("Cruising"), //Moving at a set speed
        decelerating("Decelerating"); //decelerating to approach desired point

        String name;
        travelState(String name) {this.name = name;}
    }

    private enum spinTurnState {
        notStarted("Not Started"),
        accelerating("Accelerating"), //accelerating to angular cruise speed
        cruising("Cruising"), //angular cruising speed
        decelerating("Decelerating"), //decelerating to approach tweak speed
        tweaking("Tweaking"); //speed at which final heading is corrected

        String name;
        spinTurnState(String name) {this.name = name;}
    }

    private Waypoint[] path;
    private boolean endRollout;
    private DriveSubsystem drive;

    // for autonomous path following
    public DriveAuto(Waypoint[] path, boolean endRollOut, DriveSubsystem drive) {
        this.drive = drive;
        this.path = path;
        this.endRollout = endRollOut;
        addRequirements(drive);
    }

    private Timer ControlTimer = new Timer();
    private superState superStateMachine;
    private spinTurnState turnState;
    private travelState driveState;
    private int nextWaypointIndex;
    private double speed;
    private double lastVelocity;
    private double encoderZeroValue;
    private boolean done = false;

    @Override
    public void initialize() {
        super.initialize();
        ControlTimer.reset();
        ControlTimer.start();
        superStateMachine = superState.spinTurning;
        driveState = travelState.stopped;
        turnState = spinTurnState.notStarted;
        nextWaypointIndex = 0;
        speed = 0;
        lastVelocity = 0;
        encoderZeroValue = (drive.getLeftEncoderPosition() + drive.getRightEncoderPosition()) / 2;
        done = false;
    }

    @Override
    public void execute() {
        if (done) return;

        Waypoint currentPoint = path[nextWaypointIndex];
        Waypoint nextPoint = path[nextWaypointIndex + 1];

        double headingToNextPoint = Math.toDegrees(Math.atan2(nextPoint.downFieldInches - currentPoint.downFieldInches,
                nextPoint.xFieldInches - currentPoint.xFieldInches)) + Config.IMUMountingAngle;

        double signum = 1;

        if (nextPoint.reverse) {
            signum = -1;
        }

        //negative = clockwise, positive = counterclockwise
        double degsLeftToTurn = Util3309.angleInMinus180To180(signum * drive.getHeadingError(headingToNextPoint));

        boolean turningLeft = degsLeftToTurn > 0;
        boolean turningRight = degsLeftToTurn < 0;

        double inchesBetweenWaypoints =
                Util3309.distanceFormula(currentPoint.xFieldInches, currentPoint.downFieldInches,
                        nextPoint.xFieldInches, nextPoint.downFieldInches);

        if (superStateMachine == superState.spinTurning) {

            if (nextPoint.reverse) {
                headingToNextPoint += 180;
            }
            double currentAngularVelocity = 0; //negative = clockwise, positive = counterclockwise

            if (turnState == spinTurnState.notStarted) {
                ControlTimer.reset();
                turnState = spinTurnState.accelerating;
            }

            // cache timer value so we use the same value in all of the following logic
            double timerValue = ControlTimer.get();

            if (turnState == spinTurnState.accelerating)   {
                if (turningLeft) {
                    currentAngularVelocity = signum * nextPoint.angAccelerationInDegsPerSec2 * timerValue;
                } else if (turningRight) {
                    currentAngularVelocity = -signum * nextPoint.angAccelerationInDegsPerSec2 * timerValue;
                }
            }

            //checks whether we should start cruising; we should have finished our acceleration phase
            //and we should be approaching our cruise velocity
            if (turnState == spinTurnState.accelerating &&
                    currentAngularVelocity >= nextPoint.maxAngularSpeedInDegsPerSec) {
                turnState = spinTurnState.cruising;
                if (turningLeft) {
                    currentAngularVelocity = signum * nextPoint.maxAngularSpeedInDegsPerSec;
                } else if (turningRight) {
                    currentAngularVelocity = - signum * nextPoint.maxAngularSpeedInDegsPerSec;
                }
            }
            if (turnState == spinTurnState.cruising) {
                if (turningLeft) {
                    currentAngularVelocity = signum * nextPoint.maxAngularSpeedInDegsPerSec;
                } else if (turningRight) {
                    currentAngularVelocity = - signum * nextPoint.maxAngularSpeedInDegsPerSec;
                }
            }
            if (turnState == spinTurnState.accelerating || turnState == spinTurnState.cruising) {
                //calculate how far we would continue to turn at current acceleration.
                double timeToDecelerate = (signum * currentAngularVelocity) / nextPoint.angDecelerationInDegsPerSec2;
                double degreesToDecelerate = signum * 0.5 * nextPoint.angDecelerationInDegsPerSec2 * timeToDecelerate * timeToDecelerate;
                //checks whether we should start decelerating; we should have completed cruising phase
                if (Math.abs(degreesToDecelerate) > Math.abs(degsLeftToTurn)) {
                    turnState = spinTurnState.decelerating;
                    lastVelocity = DriveSubsystem.encoderVelocityToDegsPerSec(drive.getLeftEncoderVelocity() +
                            drive.getRightEncoderVelocity()) / 2;
                    ControlTimer.reset();
                    timerValue = 0;
                }
            }

            if (turnState == spinTurnState.decelerating) {
                if (turningLeft) {
                    currentAngularVelocity = (lastVelocity - (signum * nextPoint.angDecelerationInDegsPerSec2 * timerValue));
                } else if (turningRight) {

                    currentAngularVelocity = - (lastVelocity - (signum * nextPoint.angDecelerationInDegsPerSec2 * timerValue));
                }
            }
            //checks that we have completed deceleration phase and are approaching our tweaking speed
            if (turnState == spinTurnState.decelerating && currentAngularVelocity < nextPoint.angCreepSpeedInDegsPerSec) {
                turnState = spinTurnState.tweaking;
            }
            if (turnState == spinTurnState.tweaking) {
                //check if correction is needed
                if (Math.abs(degsLeftToTurn) < Config.driveAutoSpinTurnToleranceDegrees) {
                    //spin Turn complete
                    drive.stop();
                    turnState = spinTurnState.notStarted;
                    superStateMachine = superState.drivingStraight;

                    // TODO: continue with path here instead of bailing after first spin turn
                    done = true;
                    return;
                } else if (degsLeftToTurn > 0) {  //turn left if we undershot
                    currentAngularVelocity = nextPoint.angCreepSpeedInDegsPerSec;
                } else {  //turn right if we overshot
                    currentAngularVelocity = - nextPoint.angCreepSpeedInDegsPerSec;
                }
            }

            double right = DriveSubsystem.degreesPerSecToEncoderVelocity(currentAngularVelocity);
            drive.setLeftRight(ControlMode.Velocity, -right, right);

            if (RobotContainer.getDriveDebug()) {
                SmartDashboard.putNumber("headingToNextPoint:", headingToNextPoint);
                SmartDashboard.putNumber("currentAngularVelocity:", currentAngularVelocity);
                SmartDashboard.putNumber("Commanded right velocity:", right);
                SmartDashboard.putNumber("degsLeftToTurn:", degsLeftToTurn);
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
            double encoderTicksLinear = (drive.getLeftEncoderPosition() + drive.getRightEncoderPosition()) / 2;
            double encoderTicksTraveled = encoderTicksLinear - encoderZeroValue;
            double inchesTraveled = DriveSubsystem.encoderCountsToInches((int) encoderTicksTraveled);

            double turnCorrection = degsLeftToTurn * Config.linearHeadingCorrectionFactor;

            if (driveState == travelState.stopped) {
                ControlTimer.reset();
                driveState = travelState.accelerating;
                encoderZeroValue = encoderTicksLinear;
            }
            if (driveState == travelState.accelerating) {
                speed = signum * nextPoint.linAccelerationInInchesPerSec2 * ControlTimer.get();
                if (signum * speed > signum * nextPoint.maxLinearSpeedInInchesPerSec) {
                    driveState = travelState.cruising;
                }
            }
            if (driveState == travelState.cruising){
                if (signum * (inchesBetweenWaypoints - inchesTraveled) < signum * speed * ControlTimer.get()) {
                    speed = signum * nextPoint.maxLinearSpeedInInchesPerSec;
                } else {
                    driveState = travelState.decelerating;
                    lastVelocity = DriveSubsystem.encoderVelocityToInchesPerSec(
                            (drive.getLeftEncoderVelocity() + drive.getRightEncoderVelocity()) / 2);
                    ControlTimer.reset();
                }
            }
            if (driveState == travelState.decelerating){

                speed = signum * (lastVelocity - nextPoint.linDecelerationInInchesPerSec2 * ControlTimer.get());
                if (signum *(inchesBetweenWaypoints - inchesTraveled) < signum * nextPoint.linearToleranceInInches) {
                    if (speed < nextPoint.linCreepSpeedInInchesPerSec) {
                        speed = signum * nextPoint.linCreepSpeedInInchesPerSec;
                    }
                } else {
                    if (nextWaypointIndex == path.length - 1 && !endRollout) {
                        //We are done with following the path, we have arrived at the destination
                        //Stop the robot
                        speed = 0;
                    }
                    if (inchesTraveled > inchesBetweenWaypoints + nextPoint.linearToleranceInInches) {
                        DriverStation.reportError("Traveled too far", true);
                    }
                    nextWaypointIndex++;
                    superStateMachine = superState.stopped;
                }
            }

            if (speed != 0 ) {
                drive.setArcade(ControlMode.Velocity, DriveSubsystem.inchesPerSecToEncoderVelocity(speed), turnCorrection);
            } else {
                //If speed is zero, then use PercentOutput so we don't apply brakes
                drive.setArcade(ControlMode.PercentOutput, 0,0);
            }

            if (RobotContainer.getDriveDebug()) {
                SmartDashboard.putString("Straight Line State:", driveState.name);
                SmartDashboard.putNumber("Path Correction:", turnCorrection);
                SmartDashboard.putNumber("Heading error:", degsLeftToTurn);
                SmartDashboard.putNumber("Throttle:", speed);
                SmartDashboard.putNumber("Distance to next waypoint:", inchesBetweenWaypoints);
            }

            //End of Drive straight code
        } else if (superStateMachine == superState.mobileTurning) {
            //Turn on a circle:
            //Find the length of the arc defined by turnRadiusInches, and determine
            //what the velocity the robot will have based on its current velocity.
            //Assume that the arc velocity is the average of the velocity of the two wheels.
            //Find out what velocity each wheel will have to maintain to achieve this arc velocity.
            //Set each wheel's velocity to these values.
            double arcLengthInInches = nextPoint.turnRadiusInches * (180 - headingToNextPoint);
            //find velocity at which the robot will travel while on the arc.

        } else if (superStateMachine == superState.stopped) {
            drive.stop();
        }

        if (nextWaypointIndex == path.length - 1) {
            done = true;
        }

        if (RobotContainer.getDriveDebug()) {
            SmartDashboard.putString("Robot Super State:", superStateMachine.name);
            SmartDashboard.putString("Straight Drive State:", driveState.name);
            SmartDashboard.putString("Spin Turning State:", turnState.name);
            SmartDashboard.putNumber("Previous velocity:", lastVelocity);
            SmartDashboard.putNumber("Path Array Index:", nextWaypointIndex);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}