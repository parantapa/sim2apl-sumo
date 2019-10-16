package agent.trigger.goal;

import nl.uu.cs.iss.ga.sim2apl.core.agent.AgentContextInterface;
import nl.uu.cs.iss.ga.sim2apl.core.agent.Goal;

public class AdjustCo2BasedSpeedMaintenanceGoal extends Goal {
    @Override
    public boolean isAchieved(AgentContextInterface agentContextInterface) {
        // Maintainance goals can never be achieved and should constantly be monitored
        return false;
    }
}
