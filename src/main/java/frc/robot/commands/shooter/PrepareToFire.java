package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepareToFire extends CommandBase {

    private ShooterSubsystem Shooter;
    private XboxController Controller;

    public PrepareToFire(ShooterSubsystem shooter, XboxController controller) {

        Shooter = shooter;
        Controller = controller;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        double topMotorDesired = Controller.getX(GenericHID.Hand.kLeft) * 24000;
        double bottomMotorDesired = Controller.getX(GenericHID.Hand.kRight) * 24000;

        if (Shooter.GetTopMotorVelocity() < topMotorDesired && Shooter.GetBottomMotorVelocity() < bottomMotorDesired) {

        }

    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
