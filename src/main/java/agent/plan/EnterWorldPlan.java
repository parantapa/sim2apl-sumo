package agent.plan;

import agent.context.CarContext;
import agent.trigger.goal.AdjustCo2BasedSpeedMaintenanceGoal;
import de.tudresden.sumo.cmd.Vehicle;
import de.tudresden.sumo.util.SumoCommand;
import nl.uu.cs.iss.ga.sim2apl.core.agent.PlanToAgentInterface;
import nl.uu.cs.iss.ga.sim2apl.core.plan.Plan;
import nl.uu.cs.iss.ga.sim2apl.core.plan.PlanExecutionError;
import sumo.EnvironmentAgentInterface;

/**
 * A plan that allows the agent to enter the SUMO simulation environment.
 *
 * Before this plan is performed, a route should have been created and the SUMO environment should have been informed
 * of that route.
 */
public class EnterWorldPlan extends Plan {
    @Override
    public Object execute(PlanToAgentInterface planToAgentInterface) throws PlanExecutionError {

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

//        planToAgentInterface.adoptGoal(new AdjustCo2BasedAccelerationMaintainanceGoal());
        planToAgentInterface.adoptGoal(new AdjustCo2BasedSpeedMaintenanceGoal());

        setFinished(true);
        return enter;
    }
}
