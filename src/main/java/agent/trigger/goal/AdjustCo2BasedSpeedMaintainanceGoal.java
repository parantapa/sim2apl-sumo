package agent.trigger.goal;

import org.uu.nl.sim2apl.core.agent.AgentContextInterface;
import org.uu.nl.sim2apl.core.agent.Goal;

public class AdjustCo2BasedSpeedMaintainanceGoal extends Goal {
    @Override
    public boolean isAchieved(AgentContextInterface agentContextInterface) {
        // Maintainance goals can never be achieved and should constantly be monitored
        return false;
    }
}
