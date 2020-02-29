package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Sleep extends CommandBase {

    private double sleepSeconds;
    private Timer timer = new Timer();

    public Sleep(double sleepSeconds) {
        this.sleepSeconds = sleepSeconds;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return timer.get() >= sleepSeconds;
    }
}
