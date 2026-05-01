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
        hubTable.hoodAngle.put(2.0, 0.01);
        hubTable.hoodAngle.put(3.0, 0.02);
        hubTable.hoodAngle.put(4.0, 0.03);
        hubTable.hoodAngle.put(5.0, 0.04);
        hubTable.hoodAngle.put(6.0, 0.05);
        hubTable.hoodAngle.put(8.0, 0.06);

        hubTable.flywheelSpeed.put(1.0, 175.0);
        hubTable.flywheelSpeed.put(2.0, 185.0);
        hubTable.flywheelSpeed.put(2.3, 190.0);
        hubTable.flywheelSpeed.put(2.7, 196.0);
        hubTable.flywheelSpeed.put(3.0, 202.0);
        hubTable.flywheelSpeed.put(3.5, 214.5);
        hubTable.flywheelSpeed.put(4.0, 225.0);
        hubTable.flywheelSpeed.put(5.0, 252.0);
        hubTable.flywheelSpeed.put(6.0, 280.0);
        hubTable.flywheelSpeed.put(8.0, 347.0);
        
        hubTable.timeOfFlight.put(1.0, 0.38);
        hubTable.timeOfFlight.put(2.0, 0.68);
        hubTable.timeOfFlight.put(3.0, 0.91);
        hubTable.timeOfFlight.put(3.5, 1.03);
        hubTable.timeOfFlight.put(4.0, 1.14);
        hubTable.timeOfFlight.put(5.0, 1.34);
        hubTable.timeOfFlight.put(6.0, 1.52);
        hubTable.timeOfFlight.put(8.0, 1.86);


        /* *****PASSING***** */
        passingTable.hoodAngle.put(-10000.0, 0.18);
        passingTable.hoodAngle.put(1.0, 0.18);
        passingTable.hoodAngle.put(8.0, 0.23);
        passingTable.hoodAngle.put(10000.0, 0.23);

        passingTable.flywheelSpeed.put(1.0, 160.0); // 194
        passingTable.flywheelSpeed.put(4.0, 200.0); // 220
        passingTable.flywheelSpeed.put(8.0, 250.0); // 298
        passingTable.flywheelSpeed.put(12.0, 380.0); // 442

        // Estimated Time of Flight for Passing by Gemini
        passingTable.timeOfFlight.put(1.0, 0.28);
        passingTable.timeOfFlight.put(4.0, 0.82);
        passingTable.timeOfFlight.put(8.0, 1.45);
        passingTable.timeOfFlight.put(12.0, 2.05);
    }
}
