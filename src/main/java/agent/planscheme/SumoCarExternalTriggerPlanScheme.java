package agent.planscheme;

import agent.context.CarContext;
import agent.plan.CreateRoutePlan;
import agent.trigger.external.LeftWorldExternalTrigger;
import org.uu.nl.sim2apl.core.agent.AgentContextInterface;
import org.uu.nl.sim2apl.core.agent.Trigger;
import org.uu.nl.sim2apl.core.plan.Plan;
import org.uu.nl.sim2apl.core.plan.PlanScheme;

public class SumoCarExternalTriggerPlanScheme implements PlanScheme {
    @Override
    public Plan instantiate(Trigger trigger, AgentContextInterface agentContextInterface) {
        Plan plan = Plan.UNINSTANTIATED;

        if(trigger instanceof LeftWorldExternalTrigger) {
            CarContext context = agentContextInterface.getContext(CarContext.class);
            context.setInWorld(false);

            plan = new CreateRoutePlan();
        }

        return plan;
    }
}
