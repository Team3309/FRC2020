package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.select.SelectCommand3309;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Waypoint;

public class DrivePath extends CommandBase{

    private Waypoint[] path;
    private DriveSubsystem drive;
    private boolean endRollout;

    private Waypoint[] testPath = {
            new Waypoint(0,0,0,false),
            new Waypoint(9, 9, 0, false)
    };

    public DrivePath(DriveSubsystem drive, boolean endRollout) {
        path = testPath;
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
        new DriveAuto(path, endRollout, drive).execute();
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
