package agent.plan;

import agent.context.CarContext;
import agent.trigger.goal.SetMaxSpeedMaintananceGoal;
import agent.trigger.goal.SetMinGapMaintananceGoal;
import de.tudresden.sumo.cmd.Vehicle;
import de.tudresden.sumo.util.SumoCommand;
import nl.uu.cs.iss.ga.sim2apl.core.agent.PlanToAgentInterface;
import nl.uu.cs.iss.ga.sim2apl.core.plan.Plan;
import nl.uu.cs.iss.ga.sim2apl.core.plan.PlanExecutionError;
import sumo.EnvironmentAgentInterface;

import java.util.logging.Logger;

/**
 * A plan that allows the agent to enter the SUMO simulation environment.
 *
 * Before this plan is performed, a route should have been created and the SUMO environment should have been informed
 * of that route.
 */
public class EnterWorldPlan extends Plan {
    private static final Logger LOG = Logger.getLogger(EnterWorldPlan.class.getName());

    @Override
    public Object execute(PlanToAgentInterface planToAgentInterface) throws PlanExecutionError {
        LOG.fine("Agent " + planToAgentInterface.getAgentID().getUuID() + " executing EnterWorldPlan plan");
        CarContext context = planToAgentInterface.getContext(CarContext.class);
        EnvironmentAgentInterface eaInterface = context.getEnvironmentAgentInterface();

        SumoCommand enter = Vehicle.add(
                context.getAgentInterface().getSumoID(),
                context.getAgentInterface().getTypeID(),
                context.getRouteID(),
                eaInterface.getEnvironmentInterface().simulationTime,
                0, 2, // TODO need to do anything here?
                eaInterface.getEnvironmentInterface().getLaneForEdge(context.getCurrentLocation())
        );

        LOG.fine("Agent " + planToAgentInterface.getAgentID().getUuID() + " entered world at route " + context.getRouteID());

        planToAgentInterface.adoptGoal(new SetMaxSpeedMaintananceGoal());
        planToAgentInterface.adoptGoal(new SetMinGapMaintananceGoal());

        setFinished(true);
        return enter;
    }
}
