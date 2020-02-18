package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//optimization in order to extend and move arm simultaneously (parallel command group does not work)
//Meant for deploying the intake completely
//we check for piston travel completion + arm being in position
//        //we have four cases:
//        //(1) arm is above or below intake minimum and intake is not extended,
//        //(2) arm has hit or is above intake minimum and the intake can extend
//        //(3) intake is extended, arm is above intake minimum
//        //(4) arm moved and intake extended.
//
//        //(1) becomes (2) with the initialization, (2) becomes (3) by extending the intake
//        //(3) becomes (4) when we lower the arm to minimum position
public class MoveArmAndExtendIntake extends CommandBase {

    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;
    private boolean hasSetToMin; //a check if we've initialized the final descent.

    public MoveArmAndExtendIntake(IntakeSubsystem intake, ArmSubsystem arm) {
        this.arm = arm;
        this.intake = intake;

        addRequirements(arm, intake);
    }

    @Override
    public void initialize() {
        //four cases, three giving the same result:
        hasSetToMin = false;

        //intake is already extended and in position

        //above the intake upper limit: go to the upper limit to speed things up
        //at the upper limit: do nothing / stay in place (which is identical to going to the upper limit)
        //below the upper limit: go to the upper limit to get out of the way
        if (intake.isExtended() && intake.isPistonTravelComplete()) {
            arm.moveToPosition(ArmSubsystem.ArmPosition.min);
            hasSetToMin = true;
        } else {
            arm.moveToPosition(ArmSubsystem.ArmPosition.intakeStowedUpperLimit);
        }
    }

    @Override
    public boolean isFinished() {
        //all logic goes in the is finished to eliminate rare race condition
        boolean isInPosition = arm.isInPosition();
        //we check for piston needing extending first. this locks out the second part of this function until this part completes.
        if (arm.isArmAboveIntakeMinimum()) {
            intake.extend();
        }

        //once intake is out of the way start, the final descent. If we've already done that on a previous cycle
        //then don't do that so we don't mess up our magic motion profile.
        if (intake.isPistonTravelComplete() && intake.isExtended() && !hasSetToMin) {
            arm.moveToPosition(ArmSubsystem.ArmPosition.min);
            hasSetToMin = true;
        }
        return arm.isInPosition() && hasSetToMin;
    }
}
