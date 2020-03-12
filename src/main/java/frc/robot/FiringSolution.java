package frc.robot;

import frc.robot.util.FiringSolutionManager;

/**---------------------------------------------------------------------------------------------------------------------
 * Manage the parameters needed for shooting from a given location
 */
public class FiringSolution {
    private String solutionName;
    private int armPosition;  // encoder count
    private int indexerSpeed;  // encoder counts per 100 ms
    private int topFlywheelSpeed;  // encoder counts per 100 ms
    private int bottomFlywheelSpeed;  // encoder counts per 100 ms
    private double distance;

    public FiringSolution(String solutionName, int armPosition,
                   int indexerSpeed, int topFlywheelSpeed, int bottomFlywheelSpeed) {
        this.solutionName = solutionName;
        this.armPosition = armPosition;
        this.indexerSpeed = indexerSpeed;
        this.topFlywheelSpeed = topFlywheelSpeed;
        this.bottomFlywheelSpeed = bottomFlywheelSpeed;
    }

    public FiringSolution(String solutionName, int armPosition,
                   int indexerSpeed, int topFlywheelSpeed, int bottomFlywheelSpeed, double distance) {
        this.solutionName = solutionName;
        this.armPosition = armPosition;
        this.indexerSpeed = indexerSpeed;
        this.topFlywheelSpeed = topFlywheelSpeed;
        this.bottomFlywheelSpeed = bottomFlywheelSpeed;
        this.distance = distance;

    }



    public int getArmPosition() { return armPosition; }
    public int getIndexerSpeed() { return indexerSpeed; }
    public int getTopFlywheelSpeed() { return topFlywheelSpeed; }
    public int getBottomFlywheelSpeed() { return bottomFlywheelSpeed; }

    public double getDistance() {
        return distance;
    }
}
