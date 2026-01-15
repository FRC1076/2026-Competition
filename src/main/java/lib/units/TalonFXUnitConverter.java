package lib.units;

/** Class that converts between SI units and TalonFX native units */
public class TalonFXUnitConverter {
    private final double conversionFactor;

    /** Default constructor with no overridden values. Converts from rotations to radians only. */
    public TalonFXUnitConverter() {
        conversionFactor = 2 * Math.PI;
    }

    /**
     * Constructor with overriden values for the conversion factor.
     * 
     * @apiNote Use TalonFXConfiguration.Feedback.RotorToSensorRatio for mismatches between the Kraken's internal encoder and an external encoder.
     * @apiNote Use TalonFXConfiguration.Feedback.FeedbackRotorOffset to account for offsets between the internal motor rotor and the mechanism.
     * @apiNote Use TalonFXConfiguration.Feedback.SensorToMechanismRatio to account for mismatches between the Kraken's internal encoder and mechanisms, such as those caused by gearing.
     * However, be sure not to use it for for unit conversions, as CTRE suggests against it (and that's the whole point of this class).
     * @param conversionFactor Factor to multiply position in rotations by, or velocity in rotations per second by
     */
    public TalonFXUnitConverter(double conversionFactor) {
        this.conversionFactor = conversionFactor;
    }

    /** Converts position from native units of rotations to SI units */
    public double toSIPos(double nativeUnits) {
        return nativeUnits * conversionFactor;
    }

    /** Converts velocity from native units of rotations per second to SI units */
    public double toSIVel(double nativeUnits) {
        return nativeUnits * conversionFactor;
    }

    /** Converts acceleration from native units of rotations per second per second to SI units */
    public double toSIAccel(double nativeUnits) {
        return nativeUnits * conversionFactor;
    }

    /** Converts jerk from native units of rotations per second per second per second to SI units */
    public double toSIJerk(double nativeUnits) {
        return nativeUnits * conversionFactor;
    }

    /** Converts position from SI units to native units of rotations */
    public double fromSIPos(double SIUnits) {
        return SIUnits / conversionFactor;
    }

    /** Converts velocity from SI units to native units of rotations per second */
    public double fromSIVel(double SIUnits) {
        return SIUnits / conversionFactor;
    }

    /** Converts acceleration from SI units to native units of rotations per second per second */
    public double fromSIAccel(double SIUnits) {
        return SIUnits / conversionFactor;
    }

    /** Converts acceleration from SI units to native units of rotations per second per second per second */
    public double fromSIJerk(double SIUnits) {
        return SIUnits / conversionFactor;
    }

    /** Converts kP constant from SI units to native units of output per error */
    public double fromSIkP(double SIUnits) {
        return SIUnits * conversionFactor;
    }

    /** Converts kI constant from SI units to native units of output per integrated error */
    public double fromSIkI(double SIUnits) {
        return SIUnits * conversionFactor;
    }

    /** Converts kD constant from SI units to native units of output per change in error */
    public double fromSIkD(double SIUnits) {
        return SIUnits * conversionFactor;
    }

    /** Returns the input value as kS does not depend on error */
    public double fromSIkS(double SIUnits) {
        return SIUnits;
    }

    /** Returns kV constant from SI units to native units of output per rotation per second */
    public double fromSIkV(double SIUnits) {
        return SIUnits * conversionFactor;
    }

    /** Returns kA constant from SI units to native native units of output per rotation per second per second */
    public double fromSIkA(double SIUnits) {
        return SIUnits * conversionFactor;
    }
}
