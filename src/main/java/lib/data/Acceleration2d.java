package lib.data;

public record Acceleration2d(
    double axMetersPerSecondSquared,
    double ayMetersPerSecondSquared,
    double alphaRadiansPerSecondSquared
) {
    public Acceleration2d(
        double axMetersPerSecondSquared,
        double ayMetersPerSecondSquared) {
            // Ignoring alpha
            this(axMetersPerSecondSquared, ayMetersPerSecondSquared, 0);
    }

    public Acceleration2d fromRobotRelativeAcceleration(double headingRadians) {
        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);

        return new Acceleration2d(
            (axMetersPerSecondSquared * cos) - (ayMetersPerSecondSquared * sin),
            (axMetersPerSecondSquared * sin) + (ayMetersPerSecondSquared * cos),
            alphaRadiansPerSecondSquared 
        );
    }
}
