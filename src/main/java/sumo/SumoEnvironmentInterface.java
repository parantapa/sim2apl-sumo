package sumo;

import com.sun.istack.Nullable;
import de.tudresden.sumo.cmd.*;
import de.tudresden.sumo.config.Constants;
import de.tudresden.sumo.util.SumoCommand;
import de.tudresden.ws.container.SumoStage;
import it.polito.appeal.traci.SumoTraciConnection;
import it.polito.appeal.traci.TraCIException;
import nl.uu.cs.iss.ga.sim2apl.core.agent.AgentID;
import nl.uu.cs.iss.ga.sim2apl.core.tick.TickHookProcessor;
import org.apache.commons.cli.CommandLine;

import java.io.File;
import java.net.URL;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * The SUMO environment interface handles communication with the environment.
 * This class opens and maintains a connection with SUMO, and is able to process
 * all requests and actions in that environment.
 * <p>
 * This class also handles logging and collection of statistics from the SUMO environment
 */
public class SumoEnvironmentInterface implements TickHookProcessor {
    private static final Logger LOG = Logger.getLogger(SumoEnvironmentInterface.class.getName());

    private static final String LOG_DIR = "output";

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
     * A Java Random object, to make random choices deterministic (if seed provided). Used by the system to shuffle
     * order of agent execution and agent action application. Used for reproducibility across compute nodes
     **/
    private final Random rnd;

    /**
     * A Java Random object, used by agents for action selection
     */
    private final Random agentRnd;

    /**
     * A set to keep track of agents active and present in the SUMO environment
     **/
    private final Set<String> activeAgentIDs = new HashSet<>();

    /**
     * The edges making up the road network
     **/
    private List<String> networkEdges;
    private List<String> routes;

    /**
     * Various parameters from the SUMO environment
     **/
    public int simulationTime = 0;

    @Deprecated
    private double averageCO2, averageMaxSpeed, averageAcceleration;

    /**
     * Observers listening to changes in the SUMO environment
     **/
    private List<EnvironmentAgentInterface> environmentObservers = new ArrayList<>();

    /**
     * For gathering statistics
     **/
    private String agentStatisticsFile = null;
    private String routesStatisticsFile = null;

    /**
     * Default constructor
     *
     * @param args The parsed command line arguments
     */
    public SumoEnvironmentInterface(CommandLine args, Random agentRandom) {
        LOG.finer("Constructing SUMO environment interface");
        this.sumoBinary = args.getOptionValue("sumo-binary");
        this.configFile = convertRelativeToAbsolutePath(args.getOptionValue("configuration-file"));
        this.netFile = args.hasOption("net-file") ? convertRelativeToAbsolutePath(args.getOptionValue("net-file")) : null;

        this.agentRnd = agentRandom;
        this.rnd = new Random();
        String systemSeed = args.getOptionValue("random-seed");
        if (systemSeed != null) {
            this.rnd.setSeed(Long.parseLong(systemSeed));
            LOG.fine("Created random object for system using seed" + systemSeed);
        } else {
            LOG.fine("No system seed provided. Random operations will really be random");
        }

        if (args.hasOption("step-length"))
            this.stepLength = args.getOptionValue("step-length");
        if (args.hasOption("collision.action"))
            this.collisionAction = args.getOptionValue("collision.action");
        if (args.hasOption("agent-statistics")) {
            if(args.getOptionValue("agent-statistics") == null) {
                LOG.info("Agent Statistics flag provided without argument specifying file. Generating statistics file name");
                this.agentStatisticsFile = generateStatisticsDirectory(args, "agents");
            } else {
                LOG.info("Agent statistics will be logged to provided file handler: " + args.getOptionValue("agent-statistics"));
                this.agentStatisticsFile = args.getOptionValue("agent-statistics");
            }
        }
        if (args.hasOption("route-statistics")) {
            if(args.getOptionValue("route-statistics") == null) {
                LOG.info("Routes Statistics flag provided without argument specifying file. Generating statistics file name");
                this.routesStatisticsFile = generateStatisticsDirectory(args, "routes");
            } else {
                LOG.info("Routes statistics will be logged to provided file handler: " + args.getOptionValue("route-statistics"));
                this.routesStatisticsFile = args.getOptionValue("route-statistics");
            }
        }

        startConnection();
    }

    @Override
    public void tickPreHook(long l) {
        LOG.fine("Tick pre-hook called");
        resetArrived();
        updateActiveAgents();
    }

    @Override
    public void tickPostHook(long l, int i, HashMap<AgentID, List<Object>> hashMap) {
        LOG.info(String.format("Tick %d took %d milliseconds. %d agents produced actions\n", l, i, hashMap.size()));
        List<AgentID> processAIDList = new ArrayList<>(hashMap.keySet());
        processAIDList.sort(Comparator.comparing(AgentID::getUuID));
        Collections.shuffle(processAIDList, this.rnd);

        for (AgentID aid : processAIDList) {
            LOG.finer("Processing list of actions for agent " + aid.getUuID());
            for (Object o : hashMap.get(aid)) {
                try {
                    this.connection.do_job_set((SumoCommand) o);
                } catch (IllegalStateException e) {
                    LOG.log(Level.SEVERE, "Could not peform job " + o.toString(), e);
                    closeConnection();
                    System.exit(10);
                } catch (Exception e) {
                    LOG.log(Level.WARNING, "Could not perform job " + o.toString(), e);
                }
            }
        }

        try {
            LOG.fine("Requesting SUMO to perform time step");
            this.connection.do_timestep();
            this.simulationTime = (int) this.connection.do_job_get(Simulation.getCurrentTime());
        } catch (Exception e) {
            LOG.log(Level.SEVERE, "An error occurred while performing the time step", e);
            System.exit(4);
        }
    }

    @Override
    public void simulationFinishedHook(long l, int i) {
        LOG.fine("Received simulation finished event from Simulation Engine. Closing SUMO connection");
        closeConnection();
        System.exit(0);
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
            if (route.edges.size() == 0) {
                LOG.finer("Route from " + sourceEdgeID + " to " + targetEdgeID + " has no edges. Returning zero route");
                route = null;
            }
        } catch (Exception e) {
            LOG.log(Level.WARNING,
                    "SUMO returned error when requesting route from edge " + sourceEdgeID + " to " + targetEdgeID,
                    e);
        }

        return route;
    }

    /**
     * Get a random edge in the road network
     *
     * @return Random edge ID
     */
    public String getRandomEdge() {
        return this.networkEdges.get(this.agentRnd.nextInt(this.networkEdges.size()));
    }

    public String getRandomRoute() {
        return this.routes.get(this.agentRnd.nextInt(this.routes.size()));
    }

    /**
     * Get a random lane on the given edge. 0 if the action could not succeed
     *
     * @param edgeID Edge to find random lane on
     * @return Lane index, or 0 if no lane could be found
     */
    public byte getLaneForEdge(String edgeID) {
        try {
            return (byte) this.agentRnd.nextInt((int) this.connection.do_job_get(Edge.getLaneNumber(edgeID)));
        } catch (Exception e) {
            LOG.log(Level.WARNING, "Could not get lane for edge " + edgeID + ". Using default lane 0", e);
            return (byte) 0;
        }
    }

    /**
     * Get the maximum speed associated to the specified lane
     */
    public double getLaneMaxSpeed(String laneID) {
        try {
            return (double) this.connection.do_job_get(Lane.getMaxSpeed(laneID));
        } catch (Exception e) {
            LOG.log(Level.WARNING, "Could not get maximum speed for lane " + laneID, e);
            return Double.POSITIVE_INFINITY;
        }
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
            LOG.log(Level.WARNING, "Error occurred performing GET job through TraaS", e);
            return null;
        } catch (IllegalStateException e) {
            LOG.log(Level.WARNING, "Error occurred performing GET in SUMO", e);
            System.exit(15);
            return null;
        } catch (Exception e) {
            LOG.log(Level.WARNING, "Exception occurred performing GET job in SUMO", e);
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
        return this.agentRnd;
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

        if (this.agentStatisticsFile != null) {
            this.connection.addOption("fcd-output", this.agentStatisticsFile);
        }
        if (this.routesStatisticsFile != null) {
            this.connection.addOption("tripinfo-output", this.routesStatisticsFile);
        }

        LOG.info("Starting SUMO with following command using connection " + this.connection.toString());

        try {
            this.connection.runServer();
            this.networkEdges = (List<String>) this.connection.do_job_get(Edge.getIDList());
            this.routes = (List<String>) this.connection.do_job_get(Route.getIDList());
            return true;
        } catch (Exception e) {
            LOG.log(Level.SEVERE, "Could not start connection with SUMO", e);
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
            LOG.info("Closing SUMO connection");
            this.connection.close();
        } else {
            LOG.info("Tried to close SUMO connection, but connection is already closed");
        }
    }

    /**
     * Generate a log file name for passing to SUMO
     *
     * @param args Parsed command line arguments
     * @return Log file name in OUTPUT directory
     */
    private String generateStatisticsDirectory(CommandLine args, String typeIdentifier) {
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd_HH'h'mm'm'ss.SSS", Locale.ENGLISH);
        String time = formatter.format(LocalDateTime.now());

        int iterations = Integer.parseInt(args.getOptionValue("number-of-iterations"));
        String iterationFileInfo = iterations > 0 ? String.format("_%d-iterations", iterations) : "";

        String identifier = new File(this.configFile).getName().toLowerCase();
        if (identifier.endsWith("sumo.cfg"))
            identifier = identifier.substring(0, identifier.length() - "sumo.cfg".length() - 1);
        if (identifier.endsWith("sumocfg"))
            identifier = identifier.substring(0, identifier.length() - "sumocfg".length() - 1);
        else if (identifier.contains("sumo"))
            identifier = identifier.substring(0, identifier.indexOf("sumo"));

        String logFile = String.format("%s/%s_%s_%d-Cars%s.%s.log",
                SumoEnvironmentInterface.LOG_DIR,
                time,
                identifier,
                Integer.parseInt(args.getOptionValue("number-of-cars")),
                iterationFileInfo,
                typeIdentifier
        );

        LOG.info("SUMO state will be logged to " + logFile);

        return logFile;
    }

    /**
     * Converts a path to an absolute path. This is used on all arguments given on the command line
     * that should be treated as files or locations on disk.
     *
     * @param configname Name of the configuration file that should be converted to an absolute path
     * @return The absolute path of the configName file/location, if it exists. Null otherwise
     */
    private String convertRelativeToAbsolutePath(String configname) {
        LOG.fine("Trying to resolve config file \"" + configname + "\"");
        URL resourceURL = getClass().getClassLoader().getResource(configname);
        if (new File(configname).exists()) {
            // Path was absolute
            return configname;
        } else if (resourceURL != null) {
            // Resource file was provided. Convert to absolute path
            return new File(resourceURL.getPath()).getAbsolutePath();
        } else {
            // File is not a resource
            Path path = new File(".").toPath().resolve(configname);
            if (path.toFile().exists()) {
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
            LOG.fine(removedAgents.size() + " agents arrived at the previous time step and have been removed from SUMO");
        } catch (Exception e) {
            LOG.log(Level.WARNING, "Exception occurred requesting list of arrived vehicles", e);
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
            if (this.activeAgentIDs.add(sumoAgentID)) {
                enteredAgents.add(sumoAgentID);
                LOG.finer("SUMO agent " + sumoAgentID + " is now in the environment");
            }
        }

        LOG.fine(this.activeAgentIDs.size() + " agents active in the environment. " +
                enteredAgents.size() + " entered during the last step");

        if (!enteredAgents.isEmpty()) {
            this.notifyAgentsEntered(enteredAgents);
        }
    }

    /**
     * Calculate the average values for certain parameters in the system.
     * <p>
     * Averages are calculated from the perspective of all active agents.
     */
    @Deprecated
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
            LOG.log(Level.WARNING, "Could not update average values", e);
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