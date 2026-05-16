package lib.ballistic;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class CommonLookupTable {
    public record ShotTable (
        /** Meters away from hub to hood angle in radians */
        InterpolatingDoubleTreeMap hoodAngle,
        /** Meters away from hub to flywheel speed in radians per second */
        InterpolatingDoubleTreeMap flywheelSpeed,
        /** Time in seconds to shoot to the hub based on distance in meters */
        InterpolatingDoubleTreeMap timeOfFlight
    ) {
        public ShotTable() {
            this(new InterpolatingDoubleTreeMap(), new InterpolatingDoubleTreeMap(), new InterpolatingDoubleTreeMap());
        }
    }

    /** Table containing shooting params for the hub */
    public static final ShotTable hubTable = new ShotTable();
    /** Table containing shooting params for passing */
    public static final ShotTable passingTable = hubTable;
    
    static {
        /* *****HUB***** */
        // Adjusted for a lower target with a flatter, lower hood angle configuration
        hubTable.hoodAngle.put(1.0, 0.00);
        hubTable.hoodAngle.put(2.0, 0.01);
        hubTable.hoodAngle.put(3.0, 0.015);
        hubTable.hoodAngle.put(4.0, 0.02);
        hubTable.hoodAngle.put(5.0, 0.025);
        hubTable.hoodAngle.put(6.0, 0.03);
        hubTable.hoodAngle.put(8.0, 0.04);

        // Reduced flywheel speeds to compensate for dropping 72 inches of height
        hubTable.flywheelSpeed.put(0.0, 120.0);
        hubTable.flywheelSpeed.put(1.0, 135.0);
        hubTable.flywheelSpeed.put(2.0, 150.0);
        hubTable.flywheelSpeed.put(2.3, 158.0);
        hubTable.flywheelSpeed.put(2.7, 165.0);
        hubTable.flywheelSpeed.put(3.0, 174.0);
        hubTable.flywheelSpeed.put(3.5, 187.0);
        hubTable.flywheelSpeed.put(4.0, 200.0);

        // Adjusted Time of Flight for the lower, faster trajectory
        hubTable.timeOfFlight.put(1.0, 0.30);
        hubTable.timeOfFlight.put(2.0, 0.55);
        hubTable.timeOfFlight.put(3.0, 0.75);
        hubTable.timeOfFlight.put(3.5, 0.85);
        hubTable.timeOfFlight.put(4.0, 0.94);
        hubTable.timeOfFlight.put(5.0, 1.12);
        hubTable.timeOfFlight.put(6.0, 1.28);
        hubTable.timeOfFlight.put(8.0, 1.58);
    }
}
