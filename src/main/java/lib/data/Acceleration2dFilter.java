package lib.data;

import edu.wpi.first.math.filter.LinearFilter;

public class Acceleration2dFilter {
    private final LinearFilter axFilter;
    private final LinearFilter ayFilter;
    private final LinearFilter alphaFilter;

    public Acceleration2dFilter(int taps) {
        axFilter = LinearFilter.movingAverage(taps);
        ayFilter = LinearFilter.movingAverage(taps);
        alphaFilter = LinearFilter.movingAverage(taps);
    }

    public void update(Acceleration2d accel) {
        axFilter.calculate(accel.axMetersPerSecondSquared());
        ayFilter.calculate(accel.ayMetersPerSecondSquared());
        alphaFilter.calculate(accel.alphaRadiansPerSecondSquared());
    }

    public Acceleration2d getFilteredAcceleration() {
        return new Acceleration2d(axFilter.lastValue(), ayFilter.lastValue(), alphaFilter.lastValue());
    }
}