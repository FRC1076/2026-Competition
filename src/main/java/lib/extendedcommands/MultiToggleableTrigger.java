package lib.extendedcommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Creates a Toggleable Trigger, which switches from true to false and false to true
 *  whenever the passed in Trigger's onTrue() is triggered.
 */
public class MultiToggleableTrigger {
    private Trigger[] baseTriggers;
    private boolean[] internalStates;

    /**
     * Creates a ToggleableTrigger.
     * 
     * @param base The trigger to use to switch the ToggleableTrigger
     */
    public MultiToggleableTrigger(Trigger... bases) {
        baseTriggers = bases;
        internalStates = new boolean[baseTriggers.length];
        for (int i = 0; i < internalStates.length; i++) {
            internalStates[i] = false;
        }

        for (int i = 0; i < baseTriggers.length; i++) {
            final int index = i;
            baseTriggers[index].onTrue(
                Commands.runOnce(() -> toggleInternalState(index))
            );
        }
    }

    public Trigger getToggledTrigger(int index) {
        return new Trigger(() -> internalStates[index]);
    }

    private void toggleInternalState(int index) {
        internalStates[index] = !internalStates[index];
    }

    public boolean getState(int index) {
        return internalStates[index];
    }

    public Trigger getBaseTrigger(int index) {
        return baseTriggers[index];
    }
}
