package frc.robot;

/**---------------------------------------------------------------------------------------------------------------------
 * Manage the parameters needed for shooting from a given location
 */
public class FiringSolution {
    private String solutionName;
    private int armPosition;  // encoder count
    private double indexerSpeed;  // encoder counts per 100 ms
    private double topFlywheelSpeed;  // encoder counts per 100 ms
    private double bottomFlywheelSpeed;  // encoder counts per 100 ms

    FiringSolution(String solutionName, int armPosition,
                   double indexerSpeed, double topFlywheelSpeed, double bottomFlywheelSpeed) {
        this.solutionName = solutionName;
        this.armPosition = armPosition;
        this.indexerSpeed = indexerSpeed;
        this.topFlywheelSpeed = topFlywheelSpeed;
        this.bottomFlywheelSpeed = bottomFlywheelSpeed;
    }

    FiringSolution(double rangeInches) {
        // amazing algorithm to come that will calculate the perfect solution for the given range!
        solutionName = "Calculated";
    }

    public int getArmPosition() { return armPosition; }
    public double getIndexerSpeed() { return indexerSpeed; }
    public double getTopFlywheelSpeed() { return topFlywheelSpeed; }
    public double getBottomFlywheelSpeed() { return bottomFlywheelSpeed; }
}
