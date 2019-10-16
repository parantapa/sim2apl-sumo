package sumo;

import com.sun.istack.NotNull;
import de.tudresden.ws.container.SumoPosition3D;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Class to store a few dirty statistics from the SUMO environment per tick per agent.
 *
 * Code is highly inefficient but for testing purposes only.
 */
public class Statistics {

    private singleCarStatistics[] statistics;
    private long largestTick = 0;

    public Statistics(int nCars) {
        this.statistics = new singleCarStatistics[nCars];
        for(int i = 0; i < nCars; i++) {
            this.statistics[i] = new singleCarStatistics();
        }
    }

    public void addStatisticsForCar(long tick, int carIndex, double speed, double acceleration, double co2Emission, SumoPosition3D position) {
        this.statistics[carIndex].addStatistics(tick, speed, acceleration, co2Emission, position);
        if(tick > largestTick) largestTick = tick;
    }

    private class singleCarStatistics{
        private List<SingleCarSingleTickStatistics> statisticsList;

        public singleCarStatistics() {
            this.statisticsList = new ArrayList<>();
        }

        public void addStatistics(long tick, double speed, double acceleration, double co2Emission, SumoPosition3D position) {
            this.statisticsList.add(new SingleCarSingleTickStatistics(tick, speed, acceleration, co2Emission, position));
        }

        @NotNull
        public String[] buildRows(int agentIndex) {
            String[] rows = new String[(int)Statistics.this.largestTick + 1];

            int index = 0;
            SingleCarSingleTickStatistics scsts = this.statisticsList.get(index);

            for(int i = 0; i <= Statistics.this.largestTick; i++) {
                if(scsts.tick == i) {
                    rows[i] = String.format("%d;%d;%f;%f;%f;%f;%f;%f\n",
                            scsts.tick,
                            agentIndex,
                            scsts.speed,
                            scsts.acceleration,
                            scsts.co2Emission,
                            scsts.position.x,
                            scsts.position.y,
                            scsts.position.z);
                    index++;
                    if(this.statisticsList.size() > index)
                        scsts = this.statisticsList.get(index);
                } else {
                    rows[i] = String.format("%d;%d;;;;;;\n",
                            i, agentIndex
                    );
                }
            }

            return rows;
        }
    }

    private class SingleCarSingleTickStatistics {
        private double speed;
        private double acceleration;
        private double co2Emission;
        private SumoPosition3D position;
        private long tick;

        public SingleCarSingleTickStatistics(long tick, double speed, double acceleration, double co2Emission, SumoPosition3D position) {
            this.tick = tick;
            this.speed = speed;
            this.acceleration = acceleration;
            this.co2Emission = co2Emission;
            this.position = position;
        }
    }

    public void createCsv(File f) {

        String[][] csvMatrix = new String[(int)this.largestTick + 1][this.statistics.length];

        for(int i = 0; i < this.statistics.length; i++) {
            String[] csvMatrixAgent = this.statistics[i].buildRows(i);
            for(int j = 0; j <= this.largestTick; j++) {
                csvMatrix[j][i] = csvMatrixAgent[j];
            }
        }

        StringBuilder b = new StringBuilder();
        b.append("tick;agent-index;speed;acceleration;co2-emission;position-x;position-y;position-z\n");

        for(int i = 0; i <= (int)this.largestTick; i++) {
            for(int j = 0; j < this.statistics.length; j++) {
                b.append(csvMatrix[i][j]);
            }
        }

        try {
            FileWriter fw = new FileWriter(f);
            fw.write(b.toString());
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
