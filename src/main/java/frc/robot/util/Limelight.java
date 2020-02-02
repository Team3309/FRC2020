package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Limelight {

    private NetworkTable table;
    private double xOffsetInches;
    private double zPlacementOffsetInches;
    private double zRotationOffsetInches;
    private String limelightName;
    private Timer latency3D = new Timer();

    public Limelight(String limelightName, double xOffsetInches,
                     double zPlacementOffsetInches, double zRotationOffsetInches)
    {
        table = NetworkTableInstance.getDefault().getTable(limelightName);
        this.limelightName = limelightName;
        this.xOffsetInches = xOffsetInches;
        this.zPlacementOffsetInches = zPlacementOffsetInches;
        this.zRotationOffsetInches = zRotationOffsetInches;
    }

    public boolean HasTarget() {
        return Get("tv") == 1.0;
    }

    public double GetTx() {
        return Get("tx");
    }

    public double GetTy() {
        return Get("ty");
    }

    public double GetSkew() {
        return Get("ts");
    }

    public double GetArea() {
        return Get("ta");
    }

    public double Get(String entryName) {
        return table.getEntry(entryName).getDouble(0.0);
    }

    public void SetPipeline(int pipeline) {
        table.getEntry("pipeline").setDouble(pipeline);
    }

    public int GetPipeline() {
        return (int) table.getEntry("pipeline").getDouble(0.0);
    }

    public void SetLed(LEDMode mode) {
        table.getEntry("ledMode").setDouble(mode.value);
    }

    public void SetCamMode(CamMode camMode) {
        if (camMode == CamMode.VisionProcessor) {
            table.getEntry("camMode").setDouble(0.0);
        } else if (camMode == CamMode.DriverCamera) {
            table.getEntry("camMode").setDouble(1.0);
        }
    }

    public enum LEDMode {
        Off(1),
        Blink(2),
        On(3);

        private int value;

        LEDMode(int value) {
            this.value = value;
        }

        public int get() {
            return value;
        }

    }

    public enum CamMode {
        VisionProcessor,
        DriverCamera
    }
}

