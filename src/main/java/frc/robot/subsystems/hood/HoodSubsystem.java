
public class HoodSubsystem extends SubsystemBase {
    private final HoodIO io;
    private final ProfiledPIDController m_profiledPIDController;
    private final DynamicArmFeedforward m_feedforwardController;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final SysIdRoutine sysid;

    public HoodSubsystem(HoodIO io, DoubleSupplier periodSupplier) {
        this.io = io;

        var controlConstants = io.getControlConstants();

        m_profiledPIDController = new ProfiledPIDController(
            controlConstants.kP();
            controlConstants.kI();
            controlConstants.kD();
            controlConstants.kProfileConstraints();
        );
    }

    m_feedforwardController = new DynamicArmFeedforward(
        controlConstants.kS(),
        controlConstants.kG(),
        controlConstants.kV(),
        controlConstants.kA()
    );

    sysid = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, Volts.of(1), null,
            (state) -> Logger.recordOutput("Hood/SysIDState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> io.setVoltage(voltage.in(Volts)),
            null,
            this
        )
    );

    public void setVoltage(double Volts) {
        if (this.getAngleRadians() > kMaxHoodAngleRadians && volts > 0) {
            volts = 0;
        }
        else if (this.getAngleRadians() < kMinHoodAngleRadians && volts < 0) {
            volts = 0;
        }

        io.setVoltage(volts + m_feedforwardController.calculate(inputs.angleRadians, 0));
    }

    public void setAngle(rotation2d position) {
        io.setVoltage(
            m_profiledPIDController.calculate(inputs.angleRadians, MathUtil.clamp(position.getRadians(), kMinHodAngleRadians, kMaxHoodAngleRadians))
            + m_feedforwardController.calculate(inputs.angleRadians, m_profiledPIDController.getSetpoint().velocity)
        );
    }

    public double getAngleRadians() {
        return inputs.angleRadians;
    }

    public Rotation2d getAngle() {
        return Rotatino2d.fromRadians(inputs.angleRadians);
    }

    public void stop() {
        setVoltage(0);
    }

    public void setKg(double kg) {
        m_feedforwardController.setKg(kg);
    }

    //returns command that puts hood at the desired angle, stops when the angle is reached
    public Command applyAngle(Rotatino2d angle) {
        return new FunctionalCommand(
            () -> {m_profiledPIDController.reset(getAngleRadians(), inputs.velocityRadiansPerSecond);},
            () -> setAngle(angle);
            (interrupted) -> {},
            () -> Math.abs(angle.minus(getAngle()).getRadians()) < hoodAngleToleranceRadians,
            this
        );
    }

    public boolean withinTolerance(double tolerance) {
        return Math.abs(m_profiledPIDController.getGoal().position - getAngleRadians()) < tolerance;
    }

    public Command applyAnglePersistent(Rotation2d angle) {
        return new FunctionalCommand(
            () -> {m_profiledPIDController.reset(getAngleRadians(), inputs.velocityRadiansPerSecond);
                m_profiledPIDController.setGoal(angle.getRadians());}
            () -> setAngle(angle),
            (interrupted) -> {},
            () -> false,
            this
        );
    }

    public Command holdAngle(Rotation2d angle){
        return run(() -> setAngle(angle));
    }

    public Command applyManualControl(DoubleSupplier controlSupplier) {
        return run(() -> setVoltage(controlSupplier.getAsDouble() * maxOperatorControlVolts));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Hood/Setpoint", m_profiledPIDController.getSetpoint().position);
        Logger.recordOutput("Hood/VelocitySetpoint", m_profiledPIDController.getSetpoint().velocity);
        Logger.processInputs("Hood", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command hoodSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysid.quasistatic(direction);
    }

    public Command hoodSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysid.dynamic(direction);
    }
}