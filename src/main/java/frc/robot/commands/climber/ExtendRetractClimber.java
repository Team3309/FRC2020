package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendRetractClimber extends CommandBase {

    Timer pistonTimer;
    ClimberSubsystem climber;
    boolean extend;

    public ExtendRetractClimber(boolean extend, ClimberSubsystem climber) {
        pistonTimer = new Timer();
        this.extend = extend;
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        pistonTimer.start();
        if (extend) {
            climber.deployClimber();
        } else {
            climber.retractClimber();
        }
    }

    @Override
    public boolean isFinished() {
        return pistonTimer.get() >= Config.climberDeployTime;
    }
}
