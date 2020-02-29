package frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraight extends CommandBase {

    private double inches;
    private DriveSubsystem drive;

    public DriveStraight(double inches, DriveSubsystem drive) {
        this.drive = drive;
        this.inches = inches;
    }

    @Override
    public void initialize() {
        double encoderLength = DriveSubsystem.inchesToEncoderCounts(inches);
        double leftTarget = drive.getLeftEncoderPosition() + encoderLength;
        double rightTarget = drive.getRightEncoderPosition() + encoderLength;

        // This will never work right because the drive PID is tuned for speed, not position!
        // Also, using position control will never drive straight.
        // Better to use DriveAuto once it has been debugged/calibrated for the 2020 frame.
        drive.setLeftRight(ControlMode.MotionMagic, leftTarget, rightTarget);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setLeftRight(ControlMode.PercentOutput, 0, 0);
    }

    @Override
    public boolean isFinished()
    {
        return Math.abs(drive.GetLeftClosedLoopError()) < 5000;
    }
}
