package sumo;

import com.sun.istack.Nullable;
import de.tudresden.sumo.cmd.Edge;
import de.tudresden.sumo.cmd.Simulation;
import de.tudresden.sumo.cmd.Vehicle;
import de.tudresden.sumo.config.Constants;
import de.tudresden.sumo.util.SumoCommand;
import de.tudresden.ws.container.SumoPosition3D;
import de.tudresden.ws.container.SumoStage;
import it.polito.appeal.traci.SumoTraciConnection;
import it.polito.appeal.traci.TraCIException;
import org.apache.commons.cli.CommandLine;
import org.uu.nl.sim2apl.core.agent.AgentID;
import org.uu.nl.sim2apl.core.tick.TickHookProcessor;

import java.io.File;
import java.net.URI;
import java.net.URL;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.util.*;

/**
 * The SUMO environment interface handles communication with the environment.
 * This class opens and maintains a connection with SUMO, and is able to process
 * all requests and actions in that environment.
 * <p>
 * This class also handles logging and collection of statistics from the SUMO environment
 */
public class SumoEnvironmentInterface implements TickHookProcessor {

    private SumoTraciConnection connection;

    /**
     * Various CMD args for starting the SUMO environment
     **/
    private final String sumoBinary;
    private final String configFile;
    private final String netFile;
    private String collisionAction = "none";
    private String stepLength = "1";

    /**
     * A Java Random object, to make random choices deterministic (if seed provided)
     **/
    private final Random rnd;

    /**
     * A set to keep track of agents active and present in the SUMO environment
     **/
    private final Set<String> activeAgentIDs = new HashSet<>();

    /**
     * The edges making up the road network
     **/
    private List<String> networkEdges;

    /**
     * Various parameters from the SUMO environment
     **/
    public int simulationTime = 0;
    private double averageCO2, averageMaxSpeed, averageAcceleration;

    /**
     * Observers listening to changes in the SUMO environment
     **/
    private List<EnvironmentAgentInterface> environmentObservers = new ArrayList<>();

    /**
     * For gathering statistics
     **/
    private Statistics statistics;
    private File statisticsFile = null;

    /**
     * Default constructor
     *
     * @param args The parsed command line arguments
     */
    public SumoEnvironmentInterface(CommandLine args) {

        this.sumoBinary = args.getOptionValue("sumo-binary");
        this.configFile = convertRelativeToAbsolutePath(args.getOptionValue("configuration-file"));
        this.netFile = args.hasOption("net-file") ? convertRelativeToAbsolutePath(args.getOptionValue("net-file")) : null;

        String seed = args.getOptionValue("random-seed");
        this.rnd = new Random();
        if (seed != null) {
            this.rnd.setSeed(Long.parseLong(seed));
        }

        if (args.hasOption("step-length"))
            this.stepLength = args.getOptionValue("step-length");
        if (args.hasOption("collision.action"))
            this.collisionAction = args.getOptionValue("collision.action");
        if (args.hasOption("statistics-file"))
            this.statisticsFile = new File(args.getOptionValue("statistics-file"));

        startConnection();

        if (this.statisticsFile != null)
            this.statistics = new Statistics(Integer.parseInt(args.getOptionValue("number-of-cars")));
    }


    @Override
    public void tickPreHook(long l) {
        resetArrived();
        updateActiveAgents();
        updateAverageValues();
        System.out.format("Average speed: %.2f\t\t Average acceleration: %.2f\t\tAverage CO2: %.2f\n",
                this.averageMaxSpeed, this.averageAcceleration, this.averageCO2);
    }

    @Override
    public void tickPostHook(long l, int i, HashMap<AgentID, List<Object>> hashMap) {
        System.out.format("Tick %d took %d milliseconds. %d agents produced actions\n", l, i, hashMap.size());

        for (AgentID aid : hashMap.keySet()) {
            for (Object o : hashMap.get(aid)) {
                try {
                    this.connection.do_job_set((SumoCommand) o);
                } catch (IllegalStateException e) {
                    System.err.println("Could not peform job " + o.toString());
                    System.err.println(e.getLocalizedMessage());
                    closeConnection();
                    System.exit(10);
                } catch (Exception e) {
                    System.err.println("Could not perform job " + o.toString());
                }
            }
        }

        try {
            this.connection.do_timestep();
            this.simulationTime = (int) this.connection.do_job_get(Simulation.getCurrentTime());
        } catch (Exception e) {
            System.err.println("Error performing time step");
            System.err.println(e.getLocalizedMessage());
            System.exit(4);
        }

        if (this.statisticsFile != null) updateStats(l);
    }

    @Override
    public void simulationFinishedHook(long l, int i) {
        closeConnection();
        if (this.statisticsFile != null) {
            this.statistics.createCsv(this.statisticsFile);
        }
    }

    /**
     * Verify that an agent is still in the environment
     *
     * @param sumoAgentID Agent ID of the sumo agent
     * @return True iff agent is still in the environment
     */
    public boolean isAgentActive(String sumoAgentID) {
        return this.activeAgentIDs.contains(sumoAgentID);
    }

    /**
     * Try to find a route between two edges in the network. Returns null if no route can be found
     *
     * @param sourceEdgeID ID of the edge the route should start from
     * @param targetEdgeID ID of the intended destination edge
     * @return A SumoState, encoding a route from sourceEdgeID to targetEdgeID, if one could be found.
     * Null if no route can be found, or an error occurred.
     */
    @Nullable
    public SumoStage findRoute(String sourceEdgeID, String targetEdgeID, String vehicleType) {
        SumoStage route = null;
        try {
            route = (SumoStage) this.connection.do_job_get(
                    Simulation.findRoute(sourceEdgeID, targetEdgeID, vehicleType, this.simulationTime, Constants.ROUTING_MODE_DEFAULT)
            );
            if (route.edges.size() == 0) route = null;
        } catch (Exception e) {
            System.err.println("SUMO returned error when requesting route from edge " + sourceEdgeID + " to " + targetEdgeID + ":");
            System.err.println(e.getLocalizedMessage());
        }

        return route;
    }

    /**
     * Get a random edge in the road network
     *
     * @return Random edge ID
     */
    public String getRandomEdge() {
        return this.networkEdges.get(this.rnd.nextInt(this.networkEdges.size()));
    }

    /**
     * Get a random lane on the given edge. 0 if the action could not succeed
     *
     * @param edgeID Edge to find random lane on
     * @return Lane index, or 0 if no lane could be found
     */
    public byte getLaneForEdge(String edgeID) {
        try {
            return (byte) this.rnd.nextInt((int) this.connection.do_job_get(Edge.getLaneNumber(edgeID)));
        } catch (Exception e) {
            System.err.println("Could not get lane for edge " + edgeID);
            System.err.println(e.getLocalizedMessage());
            return (byte) 0;
        }
    }

    /**
     * Get the average CO2 emission of agents after the last executed simulation step
     *
     * @return Average CO2 emission of agents
     */
    public double getAverageCO2() {
        return averageCO2;
    }

    /**
     * Get the average maximum speed of agents after the last simulation step
     *
     * @return Average maximum speed of agents
     */
    public double getAverageMaxSpeed() {
        return averageMaxSpeed;
    }

    /**
     * Get the average acceleration of agents after the last simulation step
     *
     * @return Average acceleration of agents
     */
    public double getAverageAcceleration() {
        return averageAcceleration;
    }

    /**
     * Perform a get-request in the SUMO environment. This method will handle any get request supported by TraCI/TRAAS.
     * This method handles errors. If a request fails, no error will be thrown here.
     *
     * @param cmd SumoCommand encoding get request
     * @return Object with result to cmd if request succeeded. Nul otherwise
     */
    public Object do_job_get(SumoCommand cmd) {
        try {
            return this.connection.do_job_get(cmd);
        } catch (TraCIException e) {
            System.err.println(e.getLocalizedMessage());
            return null;
        } catch (IllegalStateException e) {
            System.err.println(e.getLocalizedMessage());
            e.printStackTrace();
            System.exit(15);
            return null;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Get the Random used by the Java-end of this simulation. When a seed is provided in the command line arguments,
     * this Random instance should be used for <i>any</i> operations requiring random.
     *
     * @return Java Random object. If a seed was provided in the startup arguments, that seed is used in this object.
     */
    public Random getRandom() {
        return this.rnd;
    }

    /**
     * Starts a connection with SUMO, using parameters set with the command line arguments, or where missing
     * using defaults
     *
     * @return Boolean indicating success status
     */
    private boolean startConnection() {
        this.connection = new SumoTraciConnection(this.sumoBinary, this.configFile);
        this.connection.addOption("step-length", this.stepLength);
        this.connection.addOption("start", "1"); // Start right away
        this.connection.addOption("collision.action", this.collisionAction);

        if (this.netFile != null)
            this.connection.addOption("net-file", this.netFile);

        try {
            this.connection.runServer();
            this.networkEdges = (List<String>) this.connection.do_job_get(Edge.getIDList());
            return true;
        } catch (Exception e) {
            System.err.println("Error trying to start a connection with SUMO:");
            System.err.println(e.getLocalizedMessage());
            System.exit(1);
            closeConnection();
            return false;
        }
    }

    /**
     * Closes the connection with the SUMO environment.
     */
    private void closeConnection() {
        if (this.connection != null && !this.connection.isClosed()) {
            this.connection.close();
        }
    }

    /**
     * Converts a path to an absolute path. This is used on all arguments given on the command line
     * that should be treated as files or locations on disk.
     *
     * @param configname Name of the configuration file that should be converted to an absolute path
     * @return The absolute path of the configName file/location, if it exists. Null otherwise
     */
    private String convertRelativeToAbsolutePath(String configname) {
        System.out.println("Trying to resolve " + configname);
        URL resourceURL = getClass().getClassLoader().getResource(configname);
        if(new File(configname).exists()) {
            // Path was absolute
            return configname;
        } else if (resourceURL != null) {
            // Resource file was provided. Convert to absolute path
            return new File(resourceURL.getPath()).getAbsolutePath();
        } else {
            // File is not a resource
            Path path = new File(".").toPath().resolve(configname);
            if(path.toFile().exists()) {
                return path.toAbsolutePath().toString();
            }
        }

        return null;
    }

    /**
     * SUMO removes cars that have arrived at their intended destination. This method checks which agents have arrived
     * and thus been removed during the last simulation step. The agent interface is notified of agents that have been
     * removed from the sumo environment
     * <p>
     * This method should be called in the pre- or post-hook of every tick
     */
    private void resetArrived() {
        List<String> removedAgents;
        try {
            removedAgents = (List<String>) this.connection.do_job_get(Simulation.getArrivedIDList());
        } catch (Exception e) {
            removedAgents = Collections.emptyList();
        }

        if (!removedAgents.isEmpty()) {
            this.activeAgentIDs.removeAll(removedAgents);
            this.notifyAgentsArrived(removedAgents);
        }
    }

    /**
     * Agents can request to enter the world themselves, but this may fail for whatever reason. This method allows
     * verifying which agents have successfully entered the simulation environment in the last simulation time step,
     * keeps track of all active agents, and notifies the agent interface of successfully entered agents.
     * <p>
     * This method should be called in the pre- or post-hook of every tick
     */
    private void updateActiveAgents() {
        List<String> presentAgents;
        List<String> enteredAgents = new ArrayList<>();
        try {
            presentAgents = (List<String>) this.connection.do_job_get(Vehicle.getIDList());
        } catch (Exception e) {
            presentAgents = Collections.emptyList();
        }

        for (String sumoAgentID : presentAgents) {
            if (this.activeAgentIDs.add(sumoAgentID))
                enteredAgents.add(sumoAgentID);
        }

        if (!enteredAgents.isEmpty()) {
            this.notifyAgentsEntered(enteredAgents);
        }
    }

    /**
     * Calculate the average values for certain parameters in the system.
     * <p>
     * Averages are calculated from the perspective of all active agents.
     */
    private void updateAverageValues() {
        List<String> vehicles;
        double avgCO2 = 0;
        double avgSpeed = 0;
        double avgAcceleration = 0;

        try {
            vehicles = (List<String>) this.connection.do_job_get(Vehicle.getIDList());
            for (String vehicle : vehicles) {
                double vCO2 = (double) this.connection.do_job_get(Vehicle.getCO2Emission(vehicle));
                double vMaxSpeed = (double) this.connection.do_job_get(Vehicle.getMaxSpeed(vehicle));
                double vMaxAcc = (double) this.connection.do_job_get(Vehicle.getAccel(vehicle));
                avgCO2 += vCO2;
                avgSpeed += vMaxSpeed;
                avgAcceleration += vMaxAcc;
            }
            this.averageCO2 = avgCO2 / vehicles.size();
            this.averageMaxSpeed = avgSpeed / vehicles.size();
            this.averageAcceleration = avgAcceleration / vehicles.size();
        } catch (Exception e) {
            System.err.println("Error trying to find average values:");
            System.err.println(e.getLocalizedMessage());
        }
    }

    /**
     * Requests various statistics from the SUMO environment for all active agents and stores these for later refference.
     * This method should be called in the pre- or post-hook of every tick
     *
     * @param tick The tick index of the just finished tick
     */
    private void updateStats(long tick) {
        try {
            List<String> agentList = (List<String>) this.connection.do_job_get(Vehicle.getIDList());
            for (String agent : agentList) {
                double speed = getDouble(Vehicle.getSpeed(agent));
                double acc = getDouble(Vehicle.getAccel(agent));
                double co2 = getDouble(Vehicle.getCO2Emission(agent));
                SumoPosition3D position3D = (SumoPosition3D) this.connection.do_job_get(Vehicle.getPosition3D(agent));
                this.statistics.addStatisticsForCar(
                        tick,
                        Integer.parseInt(agent.replaceAll("\\D+", "")),
                        speed,
                        acc,
                        co2,
                        position3D);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Helper method to automatically cast a SUMO command result to a double
     *
     * @param cmd Sumo command
     * @return Sumo command casted to double
     * @throws Exception Propegated error from TraaS job execution, or casting error
     */
    private double getDouble(SumoCommand cmd) throws Exception {
        return (double) this.connection.do_job_get(cmd);
    }

    /**
     * Helper method to automatically cast a SUMO command result to a String
     *
     * @param cmd Sumo command
     * @return Sumo command casted to String
     * @throws Exception Propegated error from TraaS job execution, or casting error
     */
    private String getString(SumoCommand cmd) throws Exception {
        return (String) this.connection.do_job_get(cmd);
    }

    /**
     * Add an agent interface as a listener to this environment
     *
     * @param listener AgentInterface that intends to listen to updates from this environment
     */
    public void addEnvironmentListener(EnvironmentAgentInterface listener) {
        if (!this.environmentObservers.contains(listener))
            this.environmentObservers.add(listener);
    }

    /**
     * Remove an agent interface as a listener from this environment
     *
     * @param listener AgentInterface that intends to stop listening to updates from this environment
     */
    public void removeEnvironmentListener(EnvironmentAgentInterface listener) {
        this.environmentObservers.remove(listener);
    }

    /**
     * Notifies all subscribed listeners of agents that have arrived in and thus been removed from the SUMO environment
     * during the last time step
     *
     * @param arrivedAgents List of SUMO agent ID's of agents that have been removed from the SUMO environment
     */
    private void notifyAgentsArrived(List<String> arrivedAgents) {
        this.environmentObservers.forEach(listener -> listener.notifyAgentsArrived(arrivedAgents));
    }

    /**
     * Notifies all subscribed listeners of agents that have successfully entered the SUMO environment in the last
     * time step
     *
     * @param enteredAgents List of SUMO agent ID's of agents that have successfully entered the SUMO environment
     */
    private void notifyAgentsEntered(List<String> enteredAgents) {
        this.environmentObservers.forEach(listener -> listener.notifyAgentsEntered(enteredAgents));
    }
}
