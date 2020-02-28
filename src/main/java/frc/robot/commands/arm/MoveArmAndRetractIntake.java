package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//An optimization that allows you to retract the intake and move the arm at the same time without them colliding,
//saving time in the competition
public class MoveArmAndRetractIntake extends CommandBase {


    private final int position;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;

    public MoveArmAndRetractIntake(int position, IntakeSubsystem intake, ArmSubsystem arm) {
        this.position = position;
        this.intake = intake;
        this.arm = arm;
    }

    public void initialize() {
        if (position < Config.armPositionIntakeStowedLimit) {
            DriverStation.reportError("Requested arm position of " + position + " would block intake", false);
        } else {
            arm.moveToPosition(position);
        }
    }

    public boolean isFinished() {



        boolean isInPosition = arm.isInPosition(); //do this here to stop rare race condition
        if (arm.isArmAboveIntakeMinimum()) intake.retract();
        return isInPosition;
    }
}
