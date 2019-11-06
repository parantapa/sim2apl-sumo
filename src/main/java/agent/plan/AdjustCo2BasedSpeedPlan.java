package agent.plan;

import agent.context.CarContext;
import de.tudresden.sumo.cmd.Vehicle;
import nl.uu.cs.iss.ga.sim2apl.core.agent.PlanToAgentInterface;
import nl.uu.cs.iss.ga.sim2apl.core.plan.Plan;
import nl.uu.cs.iss.ga.sim2apl.core.plan.PlanExecutionError;
import sumo.EnvironmentAgentInterface;

import java.util.logging.Logger;

/**
 * A plan to maximize speed while staying under the average CO2 emission of all agents in the environment
 */
public class AdjustCo2BasedSpeedPlan extends Plan {

    private static final Logger LOG = Logger.getLogger(AdjustCo2BasedSpeedPlan.class.getName());

    @Override
    public Object execute(PlanToAgentInterface planToAgentInterface) throws PlanExecutionError {
        LOG.fine("Agent " + planToAgentInterface.getAgentID().getUuID() + " executing AdjustCo2BasedSpeedPlan plan");
        CarContext context = planToAgentInterface.getContext(CarContext.class);
        EnvironmentAgentInterface eaInterface = context.getEnvironmentAgentInterface();

        if(!eaInterface.getEnvironmentInterface().isAgentActive(context.getAgentInterface().getSumoID())) {
            // Do not try to take actions in the environment if you do not exist there
            LOG.fine("Agent " + planToAgentInterface.getAgentID().getUuID() + "does not exist in environment yet. Skipping");
            return null;
        }

        double avgCO2 = eaInterface.getEnvironmentInterface().getAverageCO2();

        double currMaxSpeed = (double) eaInterface.getEnvironmentInterface()
                .do_job_get(Vehicle.getCO2Emission(context.getAgentInterface().getSumoID()));

        double myEmission = (double) eaInterface.getEnvironmentInterface()
                .do_job_get(Vehicle.getMaxSpeed(context.getAgentInterface().getSumoID()));

        context.addCo2ValueToBacklog(myEmission);

        double myAvgEmission = context.getCo2BacklogAvg();

        double speedCorrectionFactor = avgCO2 < myAvgEmission ? -1d : 1d;

        LOG.fine("Agent " + planToAgentInterface.getAgentID().getUuID() + " has current average CO2 emission of " +
                myAvgEmission + " which is " + (avgCO2 < myAvgEmission ? "larger" : "smaller") + " than the average of " + avgCO2);

        setFinished(true);

        double newSpeed = currMaxSpeed + (speedCorrectionFactor * currMaxSpeed * 0.1d);

        LOG.fine("Agent " + planToAgentInterface.getAgentID().getUuID() + " calculated new speed of " + newSpeed);

        return Vehicle.setMaxSpeed(context.getAgentInterface().getSumoID(),
                    newSpeed < .1 ? .1 : newSpeed);
    }
}
