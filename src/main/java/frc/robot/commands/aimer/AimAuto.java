package frc.robot.commands.aimer;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class AimAuto extends CommandBase {

    PIDController turnPID;

    @Override
    public void initialize() {
        turnPID = new PIDController(Constants.kAimingP, Constants.kAimingI, Constants.kAimingD);


    }

    @Override
    public void execute() {
        double turn = 0;

        //TODO: Update with RobotContainer DI
//        if (Robot.vision.limelight.hasTarget()) {
//            turn = turnPID.calculate(Robot.vision.limelight.getTx(), 0);
//        }
//
//        Robot.drive.setLeftRight(ControlMode.PercentOutput, -turn, turn);

    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}