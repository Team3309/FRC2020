package frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
    private double throttle;
    private double lastVelocity;
    private double encoderZeroValue;
    private boolean done;
    private double signum;
    private double desiredHeading;
    private Waypoint currentPoint;
    double degsLeftToTurn;

    @Override
    public void initialize() {
        ControlTimer.reset();
        ControlTimer.start();
        superStateMachine = superState.spinTurning;
        driveState = travelState.stopped;
        turnState = spinTurnState.notStarted;
        waypointIndex = 0;
        throttle = 0;
        lastVelocity = 0;
        encoderZeroValue = (drive.getLeftEncoderPosition() + drive.getRightEncoderPosition()) / 2;
        done = false;
    }

    @Override
    public void execute() {
        if (done) return;

        signum = 1;
        currentPoint = path[waypointIndex];

        if (waypointIndex == path.length - 1) {
            // on last waypoint, so turn to pose heading
            desiredHeading = currentPoint.poseDegrees;
        } else {
            // turn toward next waypoint
            Waypoint nextPoint = path[waypointIndex + 1];
            desiredHeading = Math.toDegrees(Math.atan2(nextPoint.downFieldInches - currentPoint.downFieldInches,
                    nextPoint.xFieldInches - currentPoint.xFieldInches)) + Config.IMUMountingAngle;
            if (nextPoint.reverse) {
                signum = -1;
                desiredHeading += 180;
            }
        }

        //negative = clockwise, positive = counterclockwise
        degsLeftToTurn = Util3309.angleInMinus180To180(signum * drive.getHeadingError(desiredHeading));

        if (superStateMachine == superState.spinTurning) {
            spinTurn();
        }
        if (!done && superStateMachine == superState.drivingStraight) {
            driveStraight();
        }
        if (!done && superStateMachine == superState.mobileTurning) {
            driveArc();
        } else if (superStateMachine == superState.stopped) {
            drive.stop();
        }

        if (RobotContainer.getDriveDebug()) {
            SmartDashboard.putString("DriveAuto Master State", superStateMachine.name);
            SmartDashboard.putNumber("DriveAuto waypointIndex", waypointIndex);
            SmartDashboard.putString("Spin Turning State", turnState.name);
            SmartDashboard.putString("Drive Straight State", driveState.name);
            SmartDashboard.putNumber("DriveAuto signum", signum);
            SmartDashboard.putNumber("DriveAuto desiredHeading", desiredHeading);
            SmartDashboard.putNumber("DriveAuto degsLeftToTurn", degsLeftToTurn);
        }
    }

    private boolean spinTurn() {
        boolean turningLeft = degsLeftToTurn > 0;
        boolean turningRight = degsLeftToTurn < 0;

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
            SmartDashboard.putNumber("DriveAuto angularVelocity", angularVelocity);
            SmartDashboard.putNumber("DriveAuto spinTurnVelocity", spinTurnVelocity);
            SmartDashboard.putNumber("DriveAuto lastVelocity", lastVelocity);
        }
        return false;
    }

    private boolean driveStraight() {
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
                        path[waypointIndex + 1].xFieldInches, path[waypointIndex + 1].downFieldInches);

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
            throttle = signum * currentPoint.linAccelerationInInchesPerSec2 * ControlTimer.get();
            if (signum * throttle > signum * currentPoint.maxLinearSpeedInInchesPerSec) {
                driveState = travelState.cruising;
            }
        }
        if (driveState == travelState.cruising){
            if (signum * (inchesBetweenWaypoints - inchesTraveled) < signum * throttle * ControlTimer.get()) {
                throttle = signum * currentPoint.maxLinearSpeedInInchesPerSec;
            } else {
                driveState = travelState.decelerating;
                lastVelocity = DriveSubsystem.encoderVelocityToInchesPerSec(
                        (drive.getLeftEncoderVelocity() + drive.getRightEncoderVelocity()) / 2);
                ControlTimer.reset();
            }
        }
        if (driveState == travelState.decelerating){
            throttle = signum * (lastVelocity - currentPoint.linDecelerationInInchesPerSec2 * ControlTimer.get());
            if (signum *(inchesBetweenWaypoints - inchesTraveled) > signum * currentPoint.linearToleranceInInches) {
                if (Math.abs(throttle) < currentPoint.linCreepSpeedInInchesPerSec) {
                    throttle = signum * currentPoint.linCreepSpeedInInchesPerSec;
                }
            } else {
                // done with waypoint
                if (inchesTraveled > inchesBetweenWaypoints + currentPoint.linearToleranceInInches) {
                    System.out.println("Waypoint " + waypointIndex + " overshot by " +
                            (inchesTraveled - inchesBetweenWaypoints) + " inches");
                }
                throttle = 0;
                waypointIndex++;
                if (waypointIndex == path.length - 1 && path[waypointIndex].poseDegrees == null) {
                    // last waypoint has no pose, so we don't need a final turn
                    done = true;
                    superStateMachine = superState.stopped;
                }
            }
        }

        if (throttle == 0) {
            drive.stop();
        } else {
            drive.setArcade(ControlMode.Velocity, DriveSubsystem.inchesPerSecToEncoderVelocity(throttle), turnCorrection);
        }

        if (RobotContainer.getDriveDebug()) {
            SmartDashboard.putNumber("DriveAuto throttle", throttle);
            SmartDashboard.putNumber("DriveAuto turnCorrection", turnCorrection);
            SmartDashboard.putNumber("DriveAuto inchesBetweenWaypoints", inchesBetweenWaypoints);
            SmartDashboard.putNumber("DriveAuto inchesTraveled", inchesTraveled);
            SmartDashboard.putNumber("DriveAuto lastVelocity", lastVelocity);
        }
        return false;
        //End of Drive straight code
    }

    private boolean driveArc() {
        //Turn on a circle:
        //Find the length of the arc defined by turnRadiusInches, and determine
        //what the velocity the robot will have based on its current velocity.
        //Assume that the arc velocity is the average of the velocity of the two wheels.
        //Find out what velocity each wheel will have to maintain to achieve this arc velocity.
        //Set each wheel's velocity to these values.
        double arcLengthInInches = path[waypointIndex + 1].turnRadiusInches * (180 - desiredHeading);
        //find velocity at which the robot will travel while on the arc.
        return false;
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