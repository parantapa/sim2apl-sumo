package agent.context;

import agent.SumoCar2APLAgent;
import nl.uu.cs.iss.ga.sim2apl.core.agent.Context;
import sumo.EnvironmentAgentInterface;

import java.util.LinkedList;
import java.util.List;
import java.util.OptionalDouble;

public class CarContext implements Context {

    private static final int CO2_BACKLOG_LENGTH = 10;

    private final EnvironmentAgentInterface environmentAgentInterface;
    private final SumoCar2APLAgent agentInterface;

    private String currentLocation;
    private String currentTarget;
    private List<String> currentRoute;
    private String routeID;
    private List<Double> co2Backlog = new LinkedList<>();

    private boolean inWorld = false;

    public CarContext(EnvironmentAgentInterface environmentAgentInterface, SumoCar2APLAgent agentInterface) {
        this.environmentAgentInterface = environmentAgentInterface;
        this.agentInterface = agentInterface;
    }

    public EnvironmentAgentInterface getEnvironmentAgentInterface() {
        return environmentAgentInterface;
    }

    public List<String> getCurrentRoute() {
        return currentRoute;
    }

    public String getCurrentLocation() {
        return currentLocation;
    }

    public String getCurrentTarget() {
        return currentTarget;
    }

    public SumoCar2APLAgent getAgentInterface() {
        return agentInterface;
    }

    public void updateRoute(String currentLocation, List<String> route, String routeID) {
        this.currentLocation = currentLocation;
        this.currentRoute = route;
        this.routeID = routeID;
        this.currentTarget = this.currentRoute.get(this.currentRoute.size() - 1);
    }

    public void updateLocation(String location) {
        this.currentLocation = location;
    }

    public String getRouteID() {
        return routeID;
    }

    public boolean isInWorld() {
        return inWorld;
    }

    public void setInWorld(boolean inWorld) {
        this.inWorld = inWorld;
    }

    public void addCo2ValueToBacklog(double co2_value) {
        this.co2Backlog.add(co2_value);
        if(this.co2Backlog.size() > CO2_BACKLOG_LENGTH) {
            this.co2Backlog.remove(this.co2Backlog.get(0));
        }
    }

    public double getCo2BacklogAvg() {
        OptionalDouble avg = this.co2Backlog.stream().mapToDouble(a -> a).average();
        return avg.isPresent() ? avg.getAsDouble() : 0;
    }
}
