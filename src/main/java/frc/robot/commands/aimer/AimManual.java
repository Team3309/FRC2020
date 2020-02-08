package frc.robot.commands.aimer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class AimManual extends CommandBase {

    private ArmSubsystem Arm;

    public AimManual(ArmSubsystem arm) {
        Arm = arm;
        addRequirements(arm);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() { ;
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
