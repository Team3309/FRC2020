package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.select.SelectCommand3309;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Waypoint;

public class DrivePath extends CommandBase{

    public Waypoint[] path;
    private DriveSubsystem drive;
    private boolean endRollout;

    public DrivePath(DriveSubsystem drive, boolean endRollout) {
        this.drive = drive;
        this.endRollout = endRollout;
        addRequirements(drive);
    }


    public DrivePath(Waypoint[] path, DriveSubsystem drive, boolean endRollout){
        this.path = path;
        this.drive = drive;
        this.endRollout = endRollout;
        addRequirements(drive);
    }

    @Override
    public void initialize() {

        DriverStation.reportError("DrivePath executed.", false);
    }


    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
