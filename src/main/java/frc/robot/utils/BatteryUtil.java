package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotController;
import lib.extendedcommands.CommandUtils;

public class BatteryUtil {
    private static final LinearFilter m_voltageFilter = LinearFilter.movingAverage(10);

    private static boolean alreadyBrownedOut = false;
    private static int brownoutCount = 0;

    private static void periodic() {
        m_voltageFilter.calculate(RobotController.getBatteryVoltage());

        if (RobotController.isBrownedOut()) {
            if (!alreadyBrownedOut) {
                brownoutCount++;
                alreadyBrownedOut = true;
            }
        } else {
            alreadyBrownedOut = false;
        }

        Logger.recordOutput("Battery/FilteredBatteryVoltage", getFilteredBatteryVoltage());
        Logger.recordOutput("Battery/BrownoutCount", brownoutCount);
    }

    static {
        m_voltageFilter.calculate(RobotController.getBatteryVoltage());

        CommandUtils.makePeriodic(() -> periodic(), true);
    }

    public static double getFilteredBatteryVoltage() {
        return m_voltageFilter.lastValue();
    }

    public static int getBrownoutCount() {
        return brownoutCount;
    }
}
