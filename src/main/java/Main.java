import org.apache.commons.cli.*;
import sumo.EnvironmentAgentInterface;

public class Main {

    public static void main(String[] args) {
        CommandLine parsedArguments = Main.parseArguments(args);
        EnvironmentAgentInterface environmentAgentInterface = new EnvironmentAgentInterface(parsedArguments);
    }

    private static CommandLine parseArguments(String[] args) {
        Options options = Main.createOptions();

        CommandLine parsedArguments = null;

        DefaultParser parser = new DefaultParser();

        try {
            parsedArguments = parser.parse(options, args);
        } catch (ParseException e) {
            e.printStackTrace();
            HelpFormatter helpFormatter = new HelpFormatter();
            helpFormatter.printHelp("Sim2APL SUMO", options);
            System.exit(2);
        }

        return parsedArguments;
    }

    private static Options createOptions() {
        final Option sumoBinary = Option.builder("s")
                .argName("SUMO binary")
                .hasArg()
                .required()
                .longOpt("sumo-binary")
                .desc("The exact location of the SUMO binary to execute the SUMO environment")
                .build();

        final Option config = Option.builder("c")
                .argName("Configuration file location")
                .hasArg()
                .required()
                .longOpt("configuration-file")
                .desc("Loads the named config on startup")
                .build();

        final Option netFile = Option.builder("n")
                .argName("Network file location")
                .hasArg()
                .required(false)
                .longOpt("net-file")
                .desc("Load road network description from FILE")
                .build();

        final Option nCars = Option.builder()
                .argName("number of cars")
                .hasArg()
                .longOpt("number-of-cars")
                .required()
                .type(Integer.TYPE)
                .desc("The number of cars to place in the environment")
                .build();

        final Option stepLength = Option.builder()
                .argName("Step length in seconds")
                .hasArg()
                .required(false)
                .longOpt("step-length")
                .type(Long.TYPE)
                .desc("Defines the step duration in seconds")
                .build();

        final Option collisionAction = Option.builder()
                .argName("none,warn,teleport,remove")
                .hasArg()
                .required(false)
                .longOpt("collision.action")
                .type(String.class)
                .desc("How to deal with collisions: [none,warn,teleport,remove]")
                .build();

        final Option seed = Option.builder()
                .argName("Seed")
                .hasArg()
                .longOpt("random-seed")
                .required(false)
                .type(Long.TYPE)
                .desc("The seed to use for Random, so reproducibility can be ensured")
                .build();

        final Option nIterations = Option.builder("i")
                .argName("number of iterations")
                .hasArg()
                .longOpt("number-of-iterations")
                .required(false)
                .type(Long.TYPE)
                .desc("The number of iterations / ticks the simulation should perform. If not specified, simulation will " +
                        "run until interrupted manually")
                .build();

        final Option statistics = Option.builder()
                .argName("include statistics")
                .hasArg(false)
                .longOpt("statistics")
                .required(false)
                .type(boolean.class)
                .desc("Start sumo with logging enabled")
                .build();

        final Options options = new Options();

        options.addOption(sumoBinary);
        options.addOption(config);
        options.addOption(netFile);
        options.addOption(nCars);
        options.addOption(nIterations);
        options.addOption(stepLength);
        options.addOption(collisionAction);
        options.addOption(seed);
        options.addOption(statistics);

        return options;
    }
}
