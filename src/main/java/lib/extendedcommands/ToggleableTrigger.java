// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.extendedcommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Creates a Toggleable Trigger, which switches from true to false and false to true
 *  whenever the passed in Trigger's onTrue() is triggered.
 */
public class ToggleableTrigger {
    private Trigger baseTrigger;
    private boolean internalState;

    /**
     * Creates a ToggleableTrigger.
     * 
     * @param base The trigger to use to switch the ToggleableTrigger
     */
    public ToggleableTrigger(Trigger base, boolean initialState) {
        baseTrigger = base;
        internalState = initialState;

        baseTrigger.onTrue(
            Commands.runOnce(() -> toggleInternalState())
        );
    }

    public Trigger getToggledTrigger() {
        return new Trigger(() -> internalState);
    }

    private void toggleInternalState() {
        internalState = !internalState;
    }

    public boolean getState() {
        return internalState;
    }

    public Trigger getBaseTrigger() {
        return baseTrigger;
    }
}
