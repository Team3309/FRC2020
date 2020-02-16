package frc.robot.commands.aimer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimAuto extends CommandBase {

    private DriveSubsystem Drive;
    private VisionSubsystem Vision;

    PIDController turnPID;

    public AimAuto (DriveSubsystem drive, VisionSubsystem vision) {
        Drive = drive;
        Vision = vision;
    }

    @Override
    public void initialize() {
        turnPID = new PIDController(Config.aimingP, Config.aimingI, Config.aimingD);

        addRequirements();
    }

    @Override
    public void execute() {
        double turn = 0;

        /*if (Vision.limelight.HasTarget()) {
            turn = turnPID.calculate(Vision.limelight.GetTx(), 0);
        } */

        Drive.setLeftRight(ControlMode.PercentOutput, -turn, turn);

    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
