package sumo;

import agent.SumoAPLAgent;
import agent.SumoCar2APLAgent;
import agent.context.CarContext;
import agent.plan.CreateRoutePlan;
import agent.plan.EnterWorldPlan;
import agent.planscheme.SumoCarExternalTriggerPlanScheme;
import agent.planscheme.SumoCarGoalPlanScheme;
import agent.trigger.external.EnteredWorldExternalTrigger;
import agent.trigger.external.LeftWorldExternalTrigger;
import org.apache.commons.cli.CommandLine;
import org.uu.nl.sim2apl.core.agent.Agent;
import org.uu.nl.sim2apl.core.agent.AgentArguments;
import org.uu.nl.sim2apl.core.agent.AgentID;
import org.uu.nl.sim2apl.core.fipa.FIPAMessenger;
import org.uu.nl.sim2apl.core.messaging.Messenger;
import org.uu.nl.sim2apl.core.platform.Platform;
import org.uu.nl.sim2apl.core.tick.DefaultSimulationEngine;

import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * The EnvironmentAgentInterface is the coupling between the Sim2APL platform and the SUMO environment.
 * It keeps track of creating the 2APL agents, and, for each agent, stores references to both the 2APL agent instance
 * and the SUMO ID, so the corresponding 2APL agent can be found for any SUMO agent, and vice versa.
 */
public class EnvironmentAgentInterface {

    /**
     * Basic Simulation classess
     **/
    private Platform platform;
    private Messenger messenger;
    private SumoEnvironmentInterface environmentInterface;

    private long nIterations = Long.MAX_VALUE; // TODO, engine should just use WHILE loop if 0

    /** **/
    private final int desiredNOfCars;
    private int routesCounter = 0;

    /**
     * Maps to map agents from SUMO to 2APL and vice versa
     **/
    private Map<AgentID, SumoAPLAgent> aplAgents = new HashMap<>();
    private Map<String, SumoAPLAgent> sumoAgents = new HashMap<>();

    /**
     * Default constructor. Creates the Sim2APL platform, simulation engine and environment interface.
     * Starts the simulation when everything is ready automatically
     *
     * @param parsedArguments Parsed command line arguments
     */
    public EnvironmentAgentInterface(CommandLine parsedArguments) {
        this.desiredNOfCars = Integer.parseInt(parsedArguments.getOptionValue("number-of-cars"));
        if (parsedArguments.hasOption("number-of-iterations"))
            this.nIterations = Long.parseLong(parsedArguments.getOptionValue("number-of-iterations"));

        this.messenger = new FIPAMessenger();
        this.platform = Platform.newPlatform(4, this.messenger);
        this.environmentInterface = new SumoEnvironmentInterface(parsedArguments);
        this.environmentInterface.addEnvironmentListener(this);
        createInitialAgents();

        DefaultSimulationEngine engine = new DefaultSimulationEngine(platform, this.environmentInterface, this.nIterations);
        engine.start();
    }

    /**
     * Obtain a reference to the 2APL platform
     *
     * @return Platform
     */
    public Platform getPlatform() {
        return platform;
    }

    /**
     * Obtain a reference to the environment interface
     *
     * @return Environment interface
     */
    public SumoEnvironmentInterface getEnvironmentInterface() {
        return environmentInterface;
    }

    /**
     * Create a unique route ID. This is just an incremental ID string.
     *
     * @return Unique route ID for use in SUMO
     */
    public String getNewRouteID() {
        String routeId = String.format("route-%d", this.routesCounter);
        routesCounter++;
        return routeId;
    }

    /**
     * Construct the initial set of agents, based on the number of agents specified in the command line arguments
     */
    private void createInitialAgents() {
        System.out.println("Creating " + this.desiredNOfCars + " cars");
        for (int i = 0; i < this.desiredNOfCars; i++) {
            SumoAPLAgent agent = InstantiateAgent(i);
            if (agent != null) {
                this.aplAgents.put(agent.getAgentID(), agent);
                this.sumoAgents.put(agent.getSumoID(), agent);
            }
        }
    }

    /**
     * Instantiates a new 2APL agent in the 2APL platform, and creations the SumoAPLAgent object
     *
     * @param agentIndex Index of the agent (used for incremental ID generation)
     * @return SumoAPLAgent object
     */
    private SumoAPLAgent InstantiateAgent(int agentIndex) {
        String agentID = String.format("%s-%d", SumoCar2APLAgent.TYPE_ID, agentIndex);
        SumoCar2APLAgent agentInterface = new SumoCar2APLAgent(agentID);

        AgentArguments args = new AgentArguments();
        args.addContext(new CarContext(this, agentInterface));
        args.addExternalTriggerPlanScheme(new SumoCarExternalTriggerPlanScheme());
        args.addGoalPlanScheme(new SumoCarGoalPlanScheme());
        args.addInitialPlan(new CreateRoutePlan());
        args.addInitialPlan(new EnterWorldPlan());

        Agent agent;
        try {
            AgentID id = AgentID.createEmpty();
            id.setName(agentID);
            agent = new Agent(this.platform, args, id);
            agentInterface.setAgent(agent);
        } catch (URISyntaxException e) {
            e.printStackTrace();
            return null;
        }

        return agentInterface;
    }

    /**
     * Listens to updates from the environment about agents that have arrived at their destination and thus left the
     * environment. Notifies all these agents through an external trigger.
     *
     * @param arrivedAgents SUMO agent ID's of agents that have arrived at their destination and thus been removed from
     *                      the SUMO environment
     */
    void notifyAgentsArrived(List<String> arrivedAgents) {
        arrivedAgents.forEach(ra -> {
            SumoAPLAgent agentInterface = this.sumoAgents.get(ra);
            Agent a = agentInterface.getAgent();
            a.addExternalTrigger(new LeftWorldExternalTrigger());
        });
    }

    /**
     * Listens to updates form the environment about agents that have succesfully entered the SUMO environment.
     * Notifies all these agents through an external trigger
     *
     * @param enteredAgents SUMO agent ID's of successfully entered agents
     */
    void notifyAgentsEntered(List<String> enteredAgents) {
        enteredAgents.forEach(ea -> {
            SumoAPLAgent agentInterface = this.sumoAgents.get(ea);
            Agent a = agentInterface.getAgent();
            a.addExternalTrigger(new EnteredWorldExternalTrigger());
        });
    }
}
