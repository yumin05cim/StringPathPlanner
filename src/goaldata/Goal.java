package goaldata;
import java.util.Map;


public class Goal {
	
	public Pose2D pose;
	public GoalTolerance tolerance;

	public Goal(Pose2D p, GoalTolerance t){
		this.pose = p;
		this.tolerance = t;
	}

	public String toString() {
		return "(" + pose.x + ", " + pose.y + ", " + pose.heading + ", " + tolerance.distance + ", " + tolerance.heading + ")";
	}
}
