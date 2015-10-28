package goaldata;
import java.io.File;
import java.io.FileReader;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.util.Map;
import org.ho.yaml.Yaml;

public class YAMLReader {
	
	static Map<String, Object> goalMap;
	
	public YAMLReader(File file) throws FileNotFoundException, UnformattedException {
		Object obj = Yaml.load(file);
		if(obj instanceof Map<?, ?>){
			goalMap = (Map<String, Object>)obj;
		}else{
			throw new UnformattedException();
		}
	}
	
	public List<String> getGoalNameList() {
		List<String> returnValue = new ArrayList<String>();
		for(String key : goalMap.keySet()) {
			returnValue.add(key);
		}
		return returnValue;
	}

	public Map<String, Goal> getGoalMap() {
		Map<String, Goal> map = new HashMap<String, Goal>();
		for(String key : goalMap.keySet()) {
			Map<String, Object> value = (HashMap)goalMap.get(key);
			List<Double> pose = (List)value.get("pose");
			List<Double> tolerance = (List)value.get("tolerance");
			
			double x = pose.get(0).doubleValue();
			double y = pose.get(1).doubleValue();
			double a = pose.get(2).doubleValue();
			Pose2D goalPose = new Pose2D(x, y, a);
			
			double d = tolerance.get(0).doubleValue();
			double h = tolerance.get(1).doubleValue();
			GoalTolerance goalTolerance = new GoalTolerance(d, h);
			
			Goal goal = new Goal(goalPose, goalTolerance);
			
			map.put(key, goal);
			
			//System.out.println(value);
		}
 		return map;
	}

	public static void main(String[] args) {
    	try {
    		File f = new File("coordinates.yaml");
    		YAMLReader reader = new YAMLReader(f);
    		
    		Map<String, Goal> map = reader.getGoalMap();
    		for(String key : map.keySet()) {
    			Goal goal = map.get(key);
    			System.out.println("Goal (name-" + key + ") = " + goal.toString());
    		}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

}
