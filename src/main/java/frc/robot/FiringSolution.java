package frc.robot;

/**---------------------------------------------------------------------------------------------------------------------
 * Manage the parameters needed for shooting from a given location
 */
public class FiringSolution {
    private String solutionName;
    private int armPosition;  // encoder count
    private int indexerSpeed;  // encoder counts per 100 ms
    private int topFlywheelSpeed;  // encoder counts per 100 ms
    private int bottomFlywheelSpeed;  // encoder counts per 100 ms

    FiringSolution(String solutionName, int armPosition,
                   int indexerSpeed, int topFlywheelSpeed, int bottomFlywheelSpeed) {
        this.solutionName = solutionName;
        this.armPosition = armPosition;
        this.indexerSpeed = indexerSpeed;
        this.topFlywheelSpeed = topFlywheelSpeed;
        this.bottomFlywheelSpeed = bottomFlywheelSpeed;
    }

    public FiringSolution(double rangeInches, double tx, double ty) {
        // amazing algorithm to come that will calculate the perfect solution for the given range!
        double theta = Math.toRadians(tx); // we rename these for sake of sanity with the math
        double phi = Math.toRadians(ty);
        double depthToTarget = rangeInches * Math.cos(phi) * Math.cos(theta);
        double lengthToTarget = rangeInches * Math.cos(phi) * Math.sin(theta);
        double heightToTarget = rangeInches * Math.sin(phi);

        //from here we can add value to depth height and length easily because we are relative to the back hole in its basis.
        double depthToThreePointGoal = depthToTarget + Config.fieldVisionDepthOfThreePointHoleFromVisionTarget;
        double lengthToThreePointGoal = lengthToTarget;
        double heightToThreePointGoal = heightToTarget + Config.fieldVisionHeightOfThreePointHoleFromVisionTarget;

        //
        double threePointHoleDistance = Math.sqrt(depthToThreePointGoal * depthToThreePointGoal + lengthToThreePointGoal * lengthToThreePointGoal + heightToThreePointGoal * heightToThreePointGoal);
        //WE'LL NEED THESE LATER HOLD ON TO THE FORMULAE FOR NOW
        double threePointHoleTx = Math.toDegrees(Math.asin(heightToThreePointGoal / threePointHoleDistance)); //aka adjusted phi, aka the angle we need to rotate by to be facing the 3 point goal
        double threePointHoleTy = Math.toDegrees(Math.atan2(lengthToThreePointGoal, depthToThreePointGoal)); //this and distance become the arm angle and power.

        armPosition = (int) findInterpolatedFunctionValue(threePointHoleDistance,
                Config.threePointHoleDistances, Config.threePointHoleDistances);
        topFlywheelSpeed =(int) findInterpolatedFunctionValue(threePointHoleTx,
                Config.threePointHoleDistances, Config.threePointHoleTopSpeeds);
        bottomFlywheelSpeed = (int) findInterpolatedFunctionValue(threePointHoleTy,
                Config.threePointHoleDistances, Config.threePointHoleBottomSpeeds);

        solutionName = "Calculated";
    }

    /**------------------------------------------------------------------------------------------------------------
    Finds the closest function value in y to a value to a given
     */
    private double findInterpolatedFunctionValue(double x0, double[] x, double[] y) {
        for (int i = 1; i < x.length; i++) {
            if (x[i] >= x0) {
                double d = (x0 - x[i - 1]) / (x[i] - x[i - 1]);
                return y[i] * d + y[i - 1] * (1 - d);
            }
        }
        return -1;
    }



    public int getArmPosition() { return armPosition; }
    public int getIndexerSpeed() { return indexerSpeed; }
    public int getTopFlywheelSpeed() { return topFlywheelSpeed; }
    public int getBottomFlywheelSpeed() { return bottomFlywheelSpeed; }
}
