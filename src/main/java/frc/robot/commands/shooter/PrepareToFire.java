package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepareToFire extends CommandBase {

    private ShooterSubsystem Shooter;
    private boolean isDone;

    public PrepareToFire(ShooterSubsystem shooter) {

        Shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Prepares the shooter to fire by spinning up the flywheels. If the passed ShooterSubsystem's timer
     * times out, or the shooter flywheels spin up to velocity, the command ends.
     *
     \----------------------------------------------------------------------------------------------------------------*/
    @Override
    public void execute() {

        double topMotorDesired = Config.shooterStandardVelocity;
        double bottomMotorDesired = Config.shooterStandardVelocity;
        isDone = false;

        if (Shooter.GetTopMotorVelocity() < topMotorDesired && Shooter.GetBottomMotorVelocity() < bottomMotorDesired
                && !isDone && !Shooter.IsTimedOut()) {
                Shooter.SetPowerRaw(topMotorDesired, bottomMotorDesired);
        } else if (Shooter.GetTopMotorVelocity() == topMotorDesired &&
                Shooter.GetBottomMotorVelocity() == bottomMotorDesired) {
            isDone = true;
            end();
        } else {
            end();
        }



    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
