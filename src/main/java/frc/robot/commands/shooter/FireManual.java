package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class FireManual extends CommandBase {


    public ShooterSubsystem Shooter;
    public Joystick LeftStick;
    public Joystick RightStick;


    public FireManual (ShooterSubsystem shooter, Joystick lStick, Joystick rStick) {
        addRequirements(shooter);
        Shooter = shooter;
        LeftStick = lStick;
        RightStick = rStick;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        double topMotor = LeftStick.getX();
        double bottomMotor= -RightStick.getY();

        Shooter.SpinPowerCell(topMotor, bottomMotor);
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
