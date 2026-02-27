// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.GameConstants;

public class ShiftUtil {
    // get the time (in seconds) left until next 
    public static int getSecondsRemainingInShift() {
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
    public static boolean shiftEndWarning() {
        return getSecondsRemainingInShift() <= 3;
    }

    public static boolean isOurAllianceActive(){
        boolean blueAllianceActiveFirst = getAutonWinner().equals("R"); // FMS sends who will be inactive first
        boolean areWeBlue = GameConstants.teamColor == Alliance.Blue;
        int remainingSeconds = (int) DriverStation.getMatchTime();

        if (remainingSeconds > 130 || remainingSeconds < 30){
            return true;
        } else if (remainingSeconds > 105){
            return blueAllianceActiveFirst == areWeBlue;
        } else if (remainingSeconds > 80){
            return !(blueAllianceActiveFirst == areWeBlue);
        } else if (remainingSeconds > 55){
            return blueAllianceActiveFirst == areWeBlue;
        } else {
            return !(blueAllianceActiveFirst == areWeBlue);
        }
    }

    /** Returns the hex string of the alliance color whose hub is currently active.
     *  During the transition period, it returns a lighter version of the color of the first active hub.
     */
    public static String getActiveHubColorHex() {
        if (autonWinner.isBlank()) {
            return "808080";
        }
        boolean blueAllianceActiveFirst = getAutonWinner().equals("R");
        int remainingSeconds = (int) DriverStation.getMatchTime();

        if (remainingSeconds > 130){
            return blueAllianceActiveFirst
                    ? "6495ED"
                    : "F88379";
        } else if (remainingSeconds > 105){
            return blueAllianceActiveFirst
                    ? "0000FF"
                    : "FF0000";
        } else if (remainingSeconds > 80){
            return blueAllianceActiveFirst
                    ? "FF0000"
                    : "0000FF";
        } else if (remainingSeconds > 55){
            return blueAllianceActiveFirst
                    ? "0000FF"
                    : "FF0000";
        } else if (remainingSeconds > 30) {
            return blueAllianceActiveFirst
                    ? "FF0000"
                    : "0000FF";
        } else {
            return "800080";
        }
    }

    static String autonWinner = "";
    public static String getAutonWinner() {
        if (!autonWinner.isBlank()) {
            return autonWinner;
        } else {
            autonWinner = DriverStation.getGameSpecificMessage();
            return autonWinner;
        }
    }

    public static String autonWinnerColorHex() {
        if (getAutonWinner().equals("B")) {
            return "0000FF";
        } else if (getAutonWinner().equals("R")) {
            return "FF0000";
        } else {
            return "808080"; // grey for no winner yet
        }
    }

    /** Sets the auton winner.
     * 
     * @param winner R or B based on who wins auto.
     */
    public static void setAutonWinner(String winner) {
        autonWinner = winner;
    }
}
   

