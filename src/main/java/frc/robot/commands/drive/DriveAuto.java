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
    private DriveSubsystem drive;

    // for autonomous path following
    public DriveAuto(Waypoint[] path, DriveSubsystem drive) {
        this.drive = drive;
        this.path = path;
        addRequirements(drive);
    }

    private Timer ControlTimer = new Timer();
    private superState superStateMachine;
    private spinTurnState turnState;
    private travelState driveState;
    private int waypointIndex;
    private double speed;
    private double lastVelocity;
    private double encoderZeroValue;
    private boolean done;

    @Override
    public void initialize() {
        ControlTimer.reset();
        ControlTimer.start();
        superStateMachine = superState.spinTurning;
        driveState = travelState.stopped;
        turnState = spinTurnState.notStarted;
        waypointIndex = 0;
        speed = 0;
        lastVelocity = 0;
        encoderZeroValue = (drive.getLeftEncoderPosition() + drive.getRightEncoderPosition()) / 2;
        done = false;
    }

    @Override
    public void execute() {
        if (done) return;

        double signum = 1;
        double desiredHeading;
        Waypoint currentPoint = path[waypointIndex];
        Waypoint nextPoint;

        nextPoint = null;

        if (waypointIndex == path.length - 1) {
            // on last waypoint, so turn to pose heading
            desiredHeading = currentPoint.poseDegrees;
        } else {
            // turn toward next waypoint
            nextPoint = path[waypointIndex + 1];

            desiredHeading = Math.toDegrees(Math.atan2(nextPoint.downFieldInches - currentPoint.downFieldInches,
                    nextPoint.xFieldInches - currentPoint.xFieldInches)) + Config.IMUMountingAngle;

            if (nextPoint.reverse) {
                signum = -1;
                desiredHeading += 180;
            }
        }

        //negative = clockwise, positive = counterclockwise
        double degsLeftToTurn = Util3309.angleInMinus180To180(signum * drive.getHeadingError(desiredHeading));

        boolean turningLeft = degsLeftToTurn > 0;
        boolean turningRight = degsLeftToTurn < 0;

        if (superStateMachine == superState.spinTurning) {

            double angularVelocity = 0; //negative = clockwise, positive = counterclockwise

            if (turnState == spinTurnState.notStarted) {
                ControlTimer.reset();
                turnState = spinTurnState.accelerating;
            }

            // cache timer value so we use the same value in all of the following logic
            double timerValue = ControlTimer.get();

            if (turnState == spinTurnState.accelerating)   {
                if (turningLeft) {
                    angularVelocity = signum * currentPoint.angAccelerationInDegsPerSec2 * timerValue;
                } else if (turningRight) {
                    angularVelocity = -signum * currentPoint.angAccelerationInDegsPerSec2 * timerValue;
                }
            }

            //checks whether we should start cruising; we should have finished our acceleration phase
            //and we should be approaching our cruise velocity
            if (turnState == spinTurnState.accelerating &&
                    angularVelocity >= currentPoint.maxAngularSpeedInDegsPerSec) {
                turnState = spinTurnState.cruising;
                if (turningLeft) {
                    angularVelocity = signum * currentPoint.maxAngularSpeedInDegsPerSec;
                } else if (turningRight) {
                    angularVelocity = - signum * currentPoint.maxAngularSpeedInDegsPerSec;
                }
            }
            if (turnState == spinTurnState.cruising) {
                if (turningLeft) {
                    angularVelocity = signum * currentPoint.maxAngularSpeedInDegsPerSec;
                } else if (turningRight) {
                    angularVelocity = - signum * currentPoint.maxAngularSpeedInDegsPerSec;
                }
            }
            if (turnState == spinTurnState.accelerating || turnState == spinTurnState.cruising) {
                //calculate how far we would continue to turn at current acceleration.
                double timeToDecelerate = (signum * angularVelocity) / currentPoint.angDecelerationInDegsPerSec2;
                double degreesToDecelerate = signum * 0.5 * currentPoint.angDecelerationInDegsPerSec2 * timeToDecelerate * timeToDecelerate;
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
                    angularVelocity = (lastVelocity - (signum * currentPoint.angDecelerationInDegsPerSec2 * timerValue));
                } else if (turningRight) {

                    angularVelocity = - (lastVelocity - (signum * currentPoint.angDecelerationInDegsPerSec2 * timerValue));
                }
            }
            //checks that we have completed deceleration phase and are approaching our tweaking speed
            if (turnState == spinTurnState.decelerating && angularVelocity < currentPoint.angCreepSpeedInDegsPerSec) {
                turnState = spinTurnState.tweaking;
            }
            if (turnState == spinTurnState.tweaking) {
                //check if correction is needed
                if (Math.abs(degsLeftToTurn) < Config.driveAutoSpinTurnToleranceDegrees) {
                    //spin turn complete
                    drive.stop();
                    turnState = spinTurnState.notStarted;
                    angularVelocity = 0;

                    if (waypointIndex == path.length - 1) {
                        // finished turn to pose at last waypoint
                        done = true;
                    } else {
                        // start drive to next waypoint
                        superStateMachine = superState.drivingStraight;
                    }
                } else if (degsLeftToTurn > 0) {  //turn left if we undershot
                    angularVelocity = currentPoint.angCreepSpeedInDegsPerSec;
                } else {  //turn right if we overshot
                    angularVelocity = - currentPoint.angCreepSpeedInDegsPerSec;
                }
            }

            double spinTurnVelocity = DriveSubsystem.degreesPerSecToEncoderVelocity(angularVelocity);
            if (spinTurnVelocity == 0) {
                drive.stop();
            } else {
                drive.setLeftRight(ControlMode.Velocity, -spinTurnVelocity, spinTurnVelocity);
            }

            if (RobotContainer.getDriveDebug()) {
                SmartDashboard.putNumber("desiredHeading:", desiredHeading);
                SmartDashboard.putNumber("angularVelocity:", angularVelocity);
                SmartDashboard.putNumber("spinTurnVelocity:", spinTurnVelocity);
                SmartDashboard.putNumber("degsLeftToTurn:", degsLeftToTurn);
                SmartDashboard.putString("Spin turn state:", turnState.name);
            }
        }

        if (!done && superStateMachine == superState.drivingStraight) {
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
            double inchesBetweenWaypoints =
                    Util3309.distanceFormula(currentPoint.xFieldInches, currentPoint.downFieldInches,
                            nextPoint.xFieldInches, nextPoint.downFieldInches);

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
                if (signum *(inchesBetweenWaypoints - inchesTraveled) > signum * nextPoint.linearToleranceInInches) {
                    if (Math.abs(speed) < nextPoint.linCreepSpeedInInchesPerSec) {
                        speed = signum * nextPoint.linCreepSpeedInInchesPerSec;
                    }
                } else {
                    // done with waypoint
                    if (inchesTraveled > inchesBetweenWaypoints + nextPoint.linearToleranceInInches) {
                        System.out.println("Waypoint " + waypointIndex + " overshot by " +
                                (inchesTraveled - inchesBetweenWaypoints) + " inches");
                    }
                    speed = 0;
                    waypointIndex++;
                    if (waypointIndex == path.length - 1 && path[waypointIndex].poseDegrees == null) {
                        // last waypoint has no pose, so we don't need a final turn
                        done = true;
                        superStateMachine = superState.stopped;
                    }
                }
            }

            if (speed == 0) {
                drive.stop();
            } else {
                drive.setArcade(ControlMode.Velocity, DriveSubsystem.inchesPerSecToEncoderVelocity(speed), turnCorrection);
            }

            if (RobotContainer.getDriveDebug()) {
                SmartDashboard.putString("Straight Line State:", driveState.name);
                SmartDashboard.putNumber("Path Correction:", turnCorrection);
                SmartDashboard.putNumber("Heading error:", degsLeftToTurn);
                SmartDashboard.putNumber("Throttle:", speed);
                SmartDashboard.putNumber("Distance to next waypoint:", inchesBetweenWaypoints);
            }
            //End of Drive straight code

        } else if (!done && superStateMachine == superState.mobileTurning) {
            //Turn on a circle:
            //Find the length of the arc defined by turnRadiusInches, and determine
            //what the velocity the robot will have based on its current velocity.
            //Assume that the arc velocity is the average of the velocity of the two wheels.
            //Find out what velocity each wheel will have to maintain to achieve this arc velocity.
            //Set each wheel's velocity to these values.
            double arcLengthInInches = nextPoint.turnRadiusInches * (180 - desiredHeading);
            //find velocity at which the robot will travel while on the arc.

        } else if (superStateMachine == superState.stopped) {
            drive.stop();
        }

        if (RobotContainer.getDriveDebug()) {
            SmartDashboard.putString("Robot Super State:", superStateMachine.name);
            SmartDashboard.putString("Straight Drive State:", driveState.name);
            SmartDashboard.putString("Spin Turning State:", turnState.name);
            SmartDashboard.putNumber("Previous velocity:", lastVelocity);
            SmartDashboard.putNumber("Path Array Index:", waypointIndex);
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