package agent.plan;

import agent.context.CarContext;
import de.tudresden.sumo.cmd.Route;
import de.tudresden.sumo.util.SumoCommand;
import de.tudresden.ws.container.SumoStage;
import nl.uu.cs.iss.ga.sim2apl.core.agent.PlanToAgentInterface;
import nl.uu.cs.iss.ga.sim2apl.core.plan.Plan;
import nl.uu.cs.iss.ga.sim2apl.core.plan.PlanExecutionError;
import sumo.EnvironmentAgentInterface;

/**
 * A plan to create a new random route in the SUMO environment. This plan should be executed always before
 * trying to enter the simulation environment.
 *
 * If a previous route exists, the agent will first try to find a route in the reverse direction, only falling back
 * on trying to find a route between two random edges if that reversed route could not be created. If a previous
 * route already existed, that is also an indication the agent has previously left the environment. Therefor, the
 * agent adopts the plan to enter the world right after it has created and uploaded its new route.
 */
public class CreateRoutePlan extends Plan {

    public CreateRoutePlan() { }

    @Override
    public Object execute(PlanToAgentInterface planToAgentInterface) throws PlanExecutionError {
        CarContext context = planToAgentInterface.getContext(CarContext.class);
        EnvironmentAgentInterface eaInterface = context.getEnvironmentAgentInterface();

        String routeID = eaInterface.getNewRouteID();
        SumoStage plannedRoute = null;

        String startEdgeId, targetEdgeId;

        if(context.getRouteID() != null) {
            // Try to create a route which is the reverse of the previous route that was succesful
            startEdgeId = context.getCurrentTarget();
            targetEdgeId = context.getCurrentRoute().get(0);
            plannedRoute = createRoute(eaInterface, startEdgeId, targetEdgeId);
            planToAgentInterface.adoptPlan(new EnterWorldPlan());
        } else {
            startEdgeId = null; targetEdgeId = null;
        }

        while (plannedRoute == null) {
            startEdgeId = eaInterface.getEnvironmentInterface().getRandomEdge();
            targetEdgeId = eaInterface.getEnvironmentInterface().getRandomEdge();
            plannedRoute = createRoute(eaInterface, startEdgeId, targetEdgeId);
        }

        context.updateRoute(startEdgeId, plannedRoute.edges, routeID);

        SumoCommand addRoute = Route.add(routeID, plannedRoute.edges);

        setFinished(true);
        System.out.format("Agent %s created route with ID %s and %f edges\n",
                planToAgentInterface.getAgentID().toString(),
                routeID,
                plannedRoute.length);
        return addRoute;
    }

    public SumoStage createRoute(EnvironmentAgentInterface eaInterface, String randomStartEdge, String randomDestinationEdge) {
        return eaInterface.getEnvironmentInterface().findRoute(randomStartEdge, randomDestinationEdge, "car");
    }


}
