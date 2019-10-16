package agent.planscheme;

import agent.plan.AdjustCO2BasedAccelerationPlan;
import agent.trigger.goal.AdjustCo2BasedAccelerationMaintainanceGoal;
import agent.trigger.goal.AdjustCo2BasedSpeedMaintainanceGoal;
import agent.plan.AdjustCo2BasedSpeedPlan;
import org.uu.nl.sim2apl.core.agent.AgentContextInterface;
import org.uu.nl.sim2apl.core.agent.Trigger;
import org.uu.nl.sim2apl.core.plan.Plan;
import org.uu.nl.sim2apl.core.plan.PlanScheme;

public class SumoCarGoalPlanScheme implements PlanScheme {

    @Override
    public Plan instantiate(Trigger trigger, AgentContextInterface agentContextInterface) {
        Plan plan = Plan.UNINSTANTIATED;

        if(trigger instanceof AdjustCo2BasedSpeedMaintainanceGoal) {
            plan = new AdjustCo2BasedSpeedPlan();
        } else if(trigger instanceof AdjustCo2BasedAccelerationMaintainanceGoal) {
            plan = new AdjustCO2BasedAccelerationPlan();
        }

        if(plan != Plan.UNINSTANTIATED) {
            plan.setPlanGoal(trigger);
        }

        return plan;
    }
}
