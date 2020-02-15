package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntake extends CommandBase {

    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;

    public RetractIntake(IntakeSubsystem intake, ArmSubsystem arm) {
        this.intake = intake;
        this.arm = arm;
        addRequirements(intake, arm);
    }

    @Override
    public void initialize() {
        if (arm.isArmAbovePosition(ArmSubsystem.ArmPosition.intakeStowedLimit)) {
            intake.retract();
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return intake.isPistonTravelComplete();
    }
}
