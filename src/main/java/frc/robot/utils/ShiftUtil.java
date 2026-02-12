package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class ShiftUtil {
    // get the time (in seconds) left until next shift
    public int getSecondsRemainingInShift() {
        // receives time left in match 
        int currentTime = (int) DriverStation.getMatchTime();

        // if in transition return how long is in transition 
        if ( currentTime > 130){
            return currentTime - 130;

        // if in shift 1 return how long is in shift1 
        } else if (currentTime > 105) {
            return currentTime - 105;

        // if in shift 2 return how long is in shift2 
        } else if (currentTime > 80) {
            return currentTime - 80;
        
        // if in shift 3 return how long is in shift3 
        } else if (currentTime > 55) {
            return currentTime - 55; 

        // if in shift 4 return how long is in shift4 
        } else if (currentTime > 30) {
            return currentTime - 30;

        // if in endgame return how long is in endgame 
        } else {
            return currentTime;
        }
    }

    // a method if there are less then or equal to 3 seconnds 
    public boolean shiftEndWarning() {
        return getSecondsRemainingInShift() <= 3;
    }
}
   

