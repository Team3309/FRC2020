package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ShooterSubsystem;

public class FireManual extends CommandBase {


    public ShooterSubsystem Shooter;
    public XboxController Controller;

    public boolean isDone;


    public FireManual (ShooterSubsystem shooter, XboxController controller) {
        Shooter = shooter;
        Controller = controller;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        double topSpeed = Controller.getX(GenericHID.Hand.kLeft) * 24000;
        double bottomSpeed= -Controller.getX(GenericHID.Hand.kRight) * 24000;


        Shooter.setPowerRaw(topSpeed, bottomSpeed);
    }

    public void end() {
        Shooter.stopFlywheels();
        cancel();
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

}
