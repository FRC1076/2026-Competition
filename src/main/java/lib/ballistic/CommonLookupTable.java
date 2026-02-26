package lib.ballistic;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class CommonLookupTable {
    /** Meters away from hub to hood angle in radians */
    public static final InterpolatingDoubleTreeMap distanceToHoodAngleMap = new InterpolatingDoubleTreeMap();
    /** Meters away from hub to flywheel speed in radians per second */
    public static final InterpolatingDoubleTreeMap distanceToFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    /** Time in seconds to shoot to the hub based on distance in meters */
    public static final InterpolatingDoubleTreeMap distanceToTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
        distanceToHoodAngleMap.put(1.0, 0.0);
        distanceToHoodAngleMap.put(2.0, 0.05);
        distanceToHoodAngleMap.put(3.0, 0.09);
        distanceToHoodAngleMap.put(4.0, 0.12);
        distanceToHoodAngleMap.put(5.0, 0.16);
        distanceToHoodAngleMap.put(6.0, 0.2);

        distanceToFlywheelSpeedMap.put(1.0, 200.0);
        distanceToFlywheelSpeedMap.put(2.0, 215.0);
        distanceToFlywheelSpeedMap.put(3.0, 240.0);
        distanceToFlywheelSpeedMap.put(4.0, 265.0);
        distanceToFlywheelSpeedMap.put(5.0, 290.0);
        distanceToFlywheelSpeedMap.put(6.0, 315.0);

        distanceToTimeOfFlightMap.put(5.68, 1.16);
        distanceToTimeOfFlightMap.put(4.55, 1.12);
        distanceToTimeOfFlightMap.put(3.15, 1.11);
        distanceToTimeOfFlightMap.put(1.88, 1.09);
        distanceToTimeOfFlightMap.put(1.38, 0.90);
    }
}
