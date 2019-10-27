package agent.plan;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import de.tudresden.sumo.cmd.Vehicle;
import de.tudresden.sumo.util.SumoCommand;

public class AdjustCO2BasedAccelerationPlanMessage implements PlanMessage {
    public static Gson gson = new Gson();
    
    public String sumoID;
    public double acceleration;
    
    public AdjustCO2BasedAccelerationPlanMessage() {
        this.sumoID = null;
        this.acceleration = 0.0;
    }

    public AdjustCO2BasedAccelerationPlanMessage(String sumoID, double acceleration) {
        this.sumoID = sumoID;
        this.acceleration = acceleration;
    }
    
    @Override
    public SumoCommand getSumoCommand() {
        return Vehicle.setAccel(sumoID, acceleration);
    }

    @Override
    public String toJson() {
        JsonObject message = new JsonObject();
        message.addProperty("class", "AdjustCO2BasedAccelerationPlanMessage");
        
        JsonElement instance = gson.toJsonTree(this);
        message.add("instance", instance);
        
        return gson.toJson(message);
    }
}
