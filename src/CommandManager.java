import goaldata.Goal;
import goaldata.YAMLReader;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

abstract class AbstractCommandFunction {
	protected StringPathPlannerFunc func;
	public AbstractCommandFunction(StringPathPlannerFunc func) {
		this.func = func;
	}
	
	abstract public void doCommand();
}

class GoalCommand extends AbstractCommandFunction {
	
	protected Goal goal;
	private String name;
	
	public GoalCommand(StringPathPlannerFunc func, Goal goal, String name) {
		super(func);
		this.goal = goal;
		this.name = name;
		System.out.println("GoalCommand[" + name + ", " + goal + "]");
	}

	@Override
	public void doCommand() {
		this.func.setGoal(goal);
		System.out.println("[command send] " + name);
	}	
}

/*
class ActroidFunction implements AbstractCommandFunction {
	private StringPathPlannerFunc func;
	public ActroidFunction(StringPathPlannerFunc func) {
		this.func = func;
	}
	public void doCommand() {
		func.setGoal(1.0, -0.4);
	}
}
*/

class InvalidCommandException extends Exception {}

public class CommandManager {
	
	YAMLReader yaml;
	Map<String, Goal> goalMap;	
	Map<String, AbstractCommandFunction> functionMap;

	public CommandManager(StringPathPlannerFunc func, YAMLReader yaml) {
		goalMap = yaml.getGoalMap();
		functionMap = new HashMap<String, AbstractCommandFunction>();
		
		for(String goalName : goalMap.keySet()){
			functionMap.put("goal:" + goalName, new GoalCommand(func, goalMap.get(goalName), goalName));
		}
			
		functionMap.put("plan:start", new AbstractCommandFunction(func) {
			@Override
			public void doCommand() {
				System.out.println("[command send] plan:start");
				func.onPlan();
			}
		});

		functionMap.put("plan:stop", new AbstractCommandFunction(func) {
			@Override
			public void doCommand() {
				System.out.println("[command send] plan:stop");
				func.onStopPlan();
			}
		});
		
		functionMap.put("follow:start", new AbstractCommandFunction(func) {
			@Override
			public void doCommand() {
				System.out.println("[command send] follow:start");
				func.onFollow();
			}
		});

		functionMap.put("follow:stop", new AbstractCommandFunction(func) {
			@Override
			public void doCommand() {
				System.out.println("[command send] follow:stop");
			}
		});
	}

	public void doCommand(String command) throws InvalidCommandException {
		if(functionMap.containsKey(command)) {
			functionMap.get(command).doCommand();
		} else {
			System.out.println("ERROR: invalid command");
			throw new InvalidCommandException();
		}
		
	}

}
