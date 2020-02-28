package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class LiftRobot extends CommandBase {

    ClimberSubsystem climber;
    double power;

    public LiftRobot (ClimberSubsystem climber, double power) {
        this.climber = climber;
        this.power = power;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        climber.moveWinch(power);
    }

    public void end() {
        climber.moveWinch(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
