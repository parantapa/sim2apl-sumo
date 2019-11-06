package agent.plan;

import agent.context.CarContext;
import de.tudresden.sumo.cmd.Vehicle;
import nl.uu.cs.iss.ga.sim2apl.core.agent.PlanToAgentInterface;
import nl.uu.cs.iss.ga.sim2apl.core.plan.Plan;
import nl.uu.cs.iss.ga.sim2apl.core.plan.PlanExecutionError;
import sumo.EnvironmentAgentInterface;

import java.util.logging.Logger;

/**
 * A plan to maximize acceleration while staying under the average CO2 emission of all
 * agents in the environment.
 */
public class AdjustCO2BasedAccelerationPlan extends Plan {

    private static final Logger LOG = Logger.getLogger(AdjustCO2BasedAccelerationPlan.class.getName());

    @Override
    public String execute(PlanToAgentInterface planToAgentInterface) throws PlanExecutionError {
        LOG.fine("Agent " + planToAgentInterface.getAgentID().getUuID() + " executing AdjustCO2BasedAccelerationPlan plan");
        CarContext context = planToAgentInterface.getContext(CarContext.class);
        EnvironmentAgentInterface eaInterface = context.getEnvironmentAgentInterface();

        if(!eaInterface.getEnvironmentInterface().isAgentActive(context.getAgentInterface().getSumoID())) {
            // Do not try to take actions in the environment if you do not exist there
            LOG.fine("Agent " + planToAgentInterface.getAgentID().getUuID() + "does not exist in environment yet. Skipping");
            return null;
        }

        double avgCO2 = eaInterface.getEnvironmentInterface().getAverageCO2();

        Object curCO2EmissionObject = eaInterface.getEnvironmentInterface()
                .do_job_get(Vehicle.getCO2Emission(context.getAgentInterface().getSumoID()));

        Object currMaximumAccelerationObject = eaInterface.getEnvironmentInterface().do_job_get(
                Vehicle.getAccel(context.getAgentInterface().getSumoID()));

        if(currMaximumAccelerationObject == null || curCO2EmissionObject == null) return null;

        double currentMaximumAcceleration = (double) currMaximumAccelerationObject;
        double myEmission = (double) curCO2EmissionObject;

        int accelerationCorrectionFactor = avgCO2 < myEmission ? -1 : 1;

        LOG.fine("Agent " + planToAgentInterface.getAgentID().getUuID() + " has current CO2 emission of " +
                myEmission + " which is " + (avgCO2 < myEmission ? "larger" : "smaller") + " than the average of " + avgCO2);

        setFinished(true);

        double newAcc = currentMaximumAcceleration + (accelerationCorrectionFactor * currentMaximumAcceleration * 0.1);
        AdjustCO2BasedAccelerationPlanMessage message = new AdjustCO2BasedAccelerationPlanMessage(

                context.getAgentInterface().getSumoID(),
                (newAcc < .1 ? currentMaximumAcceleration : newAcc)
        );

        LOG.fine("Agent " + planToAgentInterface.getAgentID().getUuID() + " calculated new acceleration of " + newAcc);

        return message.toJson();
    }
}
