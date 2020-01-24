package org.usfirst.frc.team3309.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Constants;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.subsystems.Drive;
import org.usfirst.frc.team3309.util.CheesyDriveHelper;

public class DriveManual extends CommandBase {

    // Creating my kinematics object: track width of 27 inches
    DifferentialDriveKinematics kinematics =
            new DifferentialDriveKinematics(Units.inchesToMeters(Constants.DRIVETRAIN_WIDTH));

    @Override
    public void initialize() {
        addRequirements(Robot.drive);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.drive)) return;

        double throttle = Robot.oi.leftStick.getX();
        double turn = Robot.oi.rightStick.getY();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(throttle, 0, turn);

        // Convert to wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        Robot.drive.setLeftRight(ControlMode.PercentOutput,
                wheelSpeeds.leftMetersPerSecond,
                wheelSpeeds.rightMetersPerSecond);
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
