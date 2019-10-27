package agent.plan;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import de.tudresden.sumo.cmd.Vehicle;
import de.tudresden.sumo.util.SumoCommand;

public class AdjustCo2BasedSpeedPlanMessage implements PlanMessage {
    public static Gson gson = new Gson();
    
    public String sumoID;
    public double speed;
    
    public AdjustCo2BasedSpeedPlanMessage() {
        this.sumoID = null;
        this.speed = 0.0;
    }

    public AdjustCo2BasedSpeedPlanMessage(String sumoID, double speed) {
        this.sumoID = sumoID;
        this.speed = speed;
    }

    @Override
    public SumoCommand getSumoCommand() {
        return Vehicle.setMaxSpeed(sumoID, speed);                 
    }
    
    @Override
    public String toJson() {
        JsonObject message = new JsonObject();
        message.addProperty("class", "AdjustCo2BasedSpeedPlanMessage");
        
        JsonElement instance = gson.toJsonTree(this);
        message.add("instance", instance);
        
        return gson.toJson(message);
    }
}
