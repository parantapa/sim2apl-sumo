package agent.plan;

import agent.context.CarContext;
import agent.trigger.goal.AdjustCo2BasedSpeedMaintenanceGoal;
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
    public String execute(PlanToAgentInterface planToAgentInterface) throws PlanExecutionError {

        CarContext context = planToAgentInterface.getContext(CarContext.class);
        EnvironmentAgentInterface eaInterface = context.getEnvironmentAgentInterface();
        
        EnterWorldPlanMessage message = new EnterWorldPlanMessage(
            context.getAgentInterface().getSumoID(),
            context.getAgentInterface().getTypeID(),
            context.getRouteID(),
            eaInterface.getEnvironmentInterface().simulationTime,
            eaInterface.getEnvironmentInterface().getLaneForEdge(context.getCurrentLocation())
        );

//        planToAgentInterface.adoptGoal(new AdjustCo2BasedAccelerationMaintainanceGoal());
        planToAgentInterface.adoptGoal(new AdjustCo2BasedSpeedMaintenanceGoal());

        setFinished(true);
        
        return message.toJson();
    }
}
