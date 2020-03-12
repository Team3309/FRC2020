package frc.robot.util;

import frc.robot.FiringSolution;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.TreeMap;

public class FiringSolutionManager {

    private static FiringSolutionManager singleton;
    private TreeMap<String, ArrayList<FiringSolution>> lookupTable;
    
    private FiringSolutionManager() {
        lookupTable = new TreeMap<>();
        singleton = this;
    }

    public void addSolution(FiringSolution toBeAdded, String... tags) {
        for (int i = 0; i < tags.length; i++) {
            if (lookupTable.containsKey(tags[i])) {
                lookupTable.get(tags[i]).add(toBeAdded);
            } else {
                ArrayList arrayList = new ArrayList<FiringSolution>();
                arrayList.add(toBeAdded);
                lookupTable.put(tags[i], arrayList);
            }
        }
    }
    
    public FiringSolution lookupFiringSolution(Double distance, String... tags) {

        for (int i = 0; i < tags.length; i++) if (!lookupTable.containsKey(tags[i])) return null;

        //we start with adding in all solutions from the first tag, then filtering out the ones not in the other tags.

        ArrayList<FiringSolution> possibleSolutions = lookupTable.get(tags[0]);
        if (tags.length > 1) {
            for (int i = 1; i < tags.length; i++) {
                ArrayList<FiringSolution> solutionsToCompare = lookupTable.get(tags[0]);
                ArrayList<FiringSolution> combinedList = new ArrayList<>();
                for (int j = 0; j < possibleSolutions.size(); j++) {
                    boolean isInBoth = false;
                    for (int k = 0; k < solutionsToCompare.size(); k++) {
                        if (possibleSolutions.get(j).equals(solutionsToCompare.get(k))) {
                            isInBoth = true;
                            break;
                        }
                    }
                    if (isInBoth) {
                        combinedList.add(possibleSolutions.get(j));
                    }
                }
                possibleSolutions = combinedList;
            }
        }
        if (possibleSolutions.size() == 0) return null;
        else if (possibleSolutions.size() == 1) return possibleSolutions.get(0);
        else if (distance != null) { //linear interpolation comes in here.
            Comparator<FiringSolution> distanceComparator =
                    (FiringSolution o1, FiringSolution o2)->{
                if (o1.getDistance() > o2.getDistance()) {
                    return 1;
                } else if (o1.getDistance() == o2.getDistance()) {
                    return 0;
                } else {
                    return -1;
                }
            };
            FiringSolution[] sortedFiringSolutions = (FiringSolution[]) possibleSolutions.toArray();
            Arrays.sort(sortedFiringSolutions, distanceComparator);
            return findInterpolatedFunctionValue(distance, sortedFiringSolutions);
        } else {
            return possibleSolutions.get(0);
        }
    }

    /**------------------------------------------------------------------------------------------------------------
     Makes a firing solution from known firing solutions
     */
    private FiringSolution findInterpolatedFunctionValue(double x0, FiringSolution[] y) {
        for (int i = 1; i < y.length; i++) {
            if (y[i].getDistance() >= x0) {
                double d = (x0 - y[i - 1].getDistance()) / (y[i].getDistance() - y[i - 1].getDistance());
                return new FiringSolution("Interpolated",
                        (int) (y[i].getArmPosition() * d + y[i - 1].getArmPosition() * (1 - d)),
                        (int) (y[i].getIndexerSpeed() * d + y[i - 1].getIndexerSpeed() * (1 - d)),
                        (int) (y[i].getTopFlywheelSpeed() * d + y[i - 1].getTopFlywheelSpeed() * (1 - d)),
                        (int) (y[i].getBottomFlywheelSpeed() * d + y[i].getBottomFlywheelSpeed() * (1 - d)));
            }
        }
        return null;
    }

    public static FiringSolutionManager getSingleton() {
        return singleton == null ? new FiringSolutionManager() : singleton;
    }
}

