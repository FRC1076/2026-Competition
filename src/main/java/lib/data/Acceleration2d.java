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
}
