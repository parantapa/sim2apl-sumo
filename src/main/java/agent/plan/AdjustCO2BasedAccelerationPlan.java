package agent.plan;

import agent.context.CarContext;
import de.tudresden.sumo.cmd.Vehicle;
import nl.uu.cs.iss.ga.sim2apl.core.agent.PlanToAgentInterface;
import nl.uu.cs.iss.ga.sim2apl.core.plan.Plan;
import nl.uu.cs.iss.ga.sim2apl.core.plan.PlanExecutionError;
import sumo.EnvironmentAgentInterface;

/**
 * A plan to maximize acceleration while staying under the average CO2 emission of all
 * agents in the environment.
 */
public class AdjustCO2BasedAccelerationPlan extends Plan {
    @Override
    public String execute(PlanToAgentInterface planToAgentInterface) throws PlanExecutionError {
        CarContext context = planToAgentInterface.getContext(CarContext.class);
        EnvironmentAgentInterface eaInterface = context.getEnvironmentAgentInterface();

        if(!eaInterface.getEnvironmentInterface().isAgentActive(context.getAgentInterface().getSumoID()))
            // Do not try to take actions in the environment if you do not exist there
            return null;

        double avgCO2 = eaInterface.getEnvironmentInterface().getAverageCO2();

        Object curCO2EmissionObject = eaInterface.getEnvironmentInterface()
                .do_job_get(Vehicle.getCO2Emission(context.getAgentInterface().getSumoID()));

        Object currMaximumAccelerationObject = eaInterface.getEnvironmentInterface().do_job_get(
                Vehicle.getAccel(context.getAgentInterface().getSumoID()));

        if(currMaximumAccelerationObject == null || curCO2EmissionObject == null) return null;

        double currentMaximumAcceleration = (double) currMaximumAccelerationObject;
        double myEmission = (double) curCO2EmissionObject;

        int accelerationCorrectionFactor = avgCO2 < myEmission ? -1 : 1;

        setFinished(true);

        double newAcc = currentMaximumAcceleration + (accelerationCorrectionFactor * currentMaximumAcceleration * 0.1);
        
        AdjustCO2BasedAccelerationPlanMessage message = new AdjustCO2BasedAccelerationPlanMessage(
                context.getAgentInterface().getSumoID(),
                (newAcc < .1 ? currentMaximumAcceleration : newAcc)
        );
        
        return message.toJson();
    }
}
