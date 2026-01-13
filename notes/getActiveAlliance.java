public class Main {
// returns which hub is active at any given point in the match
  
    public static String getActiveAlliance(String firstInactive, int remainingSeconds){
      String firstActive;

      if (firstInactive.equals("R")){
        firstActive = "B";
      } 
      
      else {
        firstActive = "R";
      }
  
      if (remainingSeconds > 130 || remainingSeconds < 30){
        return "A";
      }
      
      if (remainingSeconds > 105){
        return firstActive;
      }
        
      else if (remainingSeconds > 80){
        return firstInactive;
      }

      else if (remainingSeconds > 55){
        return firstActive;
      }

      else {
        return firstInactive;
      }
  
    }
}
