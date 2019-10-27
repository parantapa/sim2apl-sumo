package agent.plan;

import agent.context.CarContext;
import de.tudresden.sumo.cmd.Vehicle;
import nl.uu.cs.iss.ga.sim2apl.core.agent.PlanToAgentInterface;
import nl.uu.cs.iss.ga.sim2apl.core.plan.Plan;
import nl.uu.cs.iss.ga.sim2apl.core.plan.PlanExecutionError;
import sumo.EnvironmentAgentInterface;

/**
 * A plan to maximize speed while staying under the average CO2 emission of all agents in the environment
 */
public class AdjustCo2BasedSpeedPlan extends Plan {

    @Override
    public String execute(PlanToAgentInterface planToAgentInterface) throws PlanExecutionError {
        CarContext context = planToAgentInterface.getContext(CarContext.class);
        EnvironmentAgentInterface eaInterface = context.getEnvironmentAgentInterface();

        if(!eaInterface.getEnvironmentInterface().isAgentActive(context.getAgentInterface().getSumoID()))
            // Do not try to take actions in the environment if you do not exist there
            return null;

        double avgCO2 = eaInterface.getEnvironmentInterface().getAverageCO2();

        double currMaxSpeed = (double) eaInterface.getEnvironmentInterface()
                .do_job_get(Vehicle.getCO2Emission(context.getAgentInterface().getSumoID()));

        double myEmission = (double) eaInterface.getEnvironmentInterface()
                .do_job_get(Vehicle.getMaxSpeed(context.getAgentInterface().getSumoID()));

        context.addCo2ValueToBacklog(myEmission);

        double myAvgEmission = context.getCo2BacklogAvg();

        double speedCorrectionFactor = avgCO2 < myAvgEmission ? -1d : 1d;

        setFinished(true);

        double newSpeed = currMaxSpeed + (speedCorrectionFactor * currMaxSpeed * 0.1d);
                
        AdjustCo2BasedSpeedPlanMessage message = new AdjustCo2BasedSpeedPlanMessage(
                context.getAgentInterface().getSumoID(),
                newSpeed < .1 ? .1 : newSpeed
        );
        
        return message.toJson();
    }
}
