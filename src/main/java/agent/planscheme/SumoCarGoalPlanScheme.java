package agent.planscheme;

import agent.plan.AdjustCO2BasedAccelerationPlan;
import agent.plan.AdjustCo2BasedSpeedPlan;
import agent.trigger.goal.AdjustCo2BasedAccelerationMaintainanceGoal;
import agent.trigger.goal.AdjustCo2BasedSpeedMaintenanceGoal;
import nl.uu.cs.iss.ga.sim2apl.core.agent.AgentContextInterface;
import nl.uu.cs.iss.ga.sim2apl.core.agent.Trigger;
import nl.uu.cs.iss.ga.sim2apl.core.plan.Plan;
import nl.uu.cs.iss.ga.sim2apl.core.plan.PlanScheme;

public class SumoCarGoalPlanScheme implements PlanScheme {

    @Override
    public Plan instantiate(Trigger trigger, AgentContextInterface agentContextInterface) {
        Plan plan = Plan.UNINSTANTIATED;

        if(trigger instanceof AdjustCo2BasedSpeedMaintenanceGoal) {
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
