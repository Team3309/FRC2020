package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendClimber extends CommandBase {

    Timer pistonTimer;
    ClimberSubsystem climber;

    public ExtendClimber (ClimberSubsystem climber) {
        pistonTimer = new Timer();
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        pistonTimer.start();
    }

    @Override
    public void execute() {
        climber.deployClimber();
    }

    @Override
    public boolean isFinished() {
        return pistonTimer.get() >= Config.climberDeployTime;
    }
}
