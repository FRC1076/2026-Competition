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
    public static final ShotTable passingTable = new ShotTable();
    
    static {
        /* *****HUB***** */
        hubTable.hoodAngle.put(1.0, 0.0);
        hubTable.hoodAngle.put(2.0, 0.05);
        hubTable.hoodAngle.put(3.0, 0.09);
        hubTable.hoodAngle.put(4.0, 0.12);
        hubTable.hoodAngle.put(5.0, 0.14);
        hubTable.hoodAngle.put(6.0, 0.16);
        hubTable.hoodAngle.put(8.0, 0.18);

        hubTable.flywheelSpeed.put(1.0, 176.0);
        hubTable.flywheelSpeed.put(2.0, 188.0);
        hubTable.flywheelSpeed.put(2.3, 192.0);
        hubTable.flywheelSpeed.put(2.7, 196.0);
        hubTable.flywheelSpeed.put(3.0, 201.0);
        hubTable.flywheelSpeed.put(4.0, 225.0);
        hubTable.flywheelSpeed.put(5.0, 250.0);
        hubTable.flywheelSpeed.put(6.0, 275.0);
        hubTable.flywheelSpeed.put(8.0, 345.0);
        
        hubTable.timeOfFlight.put(1.0, 0.38);
        hubTable.timeOfFlight.put(2.0, 0.68);
        hubTable.timeOfFlight.put(3.0, 0.9);
        hubTable.timeOfFlight.put(3.5, 1.0);
        hubTable.timeOfFlight.put(4.0, 1.1);
        hubTable.timeOfFlight.put(5.0, 1.28);
        hubTable.timeOfFlight.put(6.0, 1.45);
        hubTable.timeOfFlight.put(8.0, 1.8);


        /* *****PASSING***** */
        passingTable.hoodAngle.put(1.0, 0.05);
        passingTable.hoodAngle.put(5.0, 0.1);
        passingTable.hoodAngle.put(8.0, 0.18);
        passingTable.hoodAngle.put(12.0, 0.25);

        passingTable.flywheelSpeed.put(1.0, 190.0); // 194
        passingTable.flywheelSpeed.put(4.0, 230.0); // 220
        passingTable.flywheelSpeed.put(8.0, 290.0); // 298
        passingTable.flywheelSpeed.put(12.0, 425.0); // 442

        // Estimated Time of Flight for Passing by Gemini
        passingTable.timeOfFlight.put(1.0, 0.28);
        passingTable.timeOfFlight.put(4.0, 0.82);
        passingTable.timeOfFlight.put(8.0, 1.45);
        passingTable.timeOfFlight.put(12.0, 2.05);
    }
}
