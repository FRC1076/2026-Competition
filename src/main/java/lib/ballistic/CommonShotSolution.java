// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.ballistic;

public record CommonShotSolution (
    double launchPitchRad,
    double launchYawRad,
    double launchSpeedRadPerSec
) {
    public static CommonShotSolution withZeroPitch(CommonShotSolution old) {
        return new CommonShotSolution(0, old.launchYawRad(), old.launchSpeedRadPerSec());
    }
}
