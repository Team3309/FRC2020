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

    //we don't really need a position parameter as the current state machine only wants to ever
    //have the intake out if we're
    public MoveArmAndExtendIntake(IntakeSubsystem intake, ArmSubsystem arm) {
        this.arm = arm;
        this.intake = intake;

        addRequirements(arm, intake);
    }

    @Override
    public void initialize() {
        //three cases, all giving the same result:

        //above the intake upper limit: go to the upper limit to speed things up
        //at the upper limit: do nothing / stay in place (which is identical to going to the upper limit)
        //below the upper limit: go to the upper limit to get out of the way
        arm.moveToPosition(ArmSubsystem.ArmPosition.intakeStowedUpperLimit);
    }

    @Override
    public boolean isFinished() {
        //all logic goes in the is finished to eliminate rare race condition
        boolean isInPosition = arm.isInPosition();
        //we check for piston needing extending first. this locks out the second part of this function until this part completes.
        if (arm.isArmAboveIntakeMinimum()) {
            intake.extend();
        }

        //piston travel is complete returns true if the arm is in the way, so we also check if we are above intake minimum
        boolean isPistonTravelComplete = intake.isPistonTravelComplete(); //do this here as well to eliminate rare race condition
        if (isPistonTravelComplete && arm.isArmAboveIntakeMinimum()) {
            arm.moveToPosition(ArmSubsystem.ArmPosition.min); //this is an intake setup command so there's nowhere else to go
        }
        //these two conditions are only true when both actions are done.
        return isPistonTravelComplete && isInPosition && arm.isInPosition();
        //check both the initial boolean we set and the current position to prevent premature exits.
    }
}
