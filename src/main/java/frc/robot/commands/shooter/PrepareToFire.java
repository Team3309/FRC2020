package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepareToFire extends CommandBase {

    private ShooterSubsystem Shooter;
    private XboxController Controller;
    private Timer ControlTimer;
    private boolean isDone;

    public PrepareToFire(ShooterSubsystem shooter, Timer ctrlTimer) {

        Shooter = shooter;
        ControlTimer = ctrlTimer;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        double topMotorDesired = Config.ShooterStandardVelocity;
        double bottomMotorDesired = Config.ShooterStandardVelocity;
        isDone = false;

        if (Shooter.GetTopMotorVelocity() < topMotorDesired && Shooter.GetBottomMotorVelocity() < bottomMotorDesired
                && !isDone) {
                Shooter.SpinPowerCell(topMotorDesired, bottomMotorDesired);
        } else if (Shooter.GetTopMotorVelocity() == topMotorDesired &&
                Shooter.GetBottomMotorVelocity() == bottomMotorDesired) {
            isDone = true;
        }

    }

    public void end() {
        Shooter.StopFlywheels();
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
