import goaldata.Goal;

import java.awt.Point;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.StringTokenizer;

import RTC.Path2D;
import RTC.PathPlanParameter;
import RTC.Point2D;
import RTC.Pose2D;
import RTC.Velocity2D;

public class StringPathPlannerFunc {
	
	private RTC.Pose2D robotPose;
	
	private RTC.Path2D path;
	
	private Pose2D goal;
	
	private StringPathPlannerImpl rtc;
	
	public StringPathPlannerFunc(StringPathPlannerImpl rtc){
		this.rtc = rtc;
	}
	
	public void setRobotPose(Pose2D pose){
		this.robotPose = pose;
	}
	
//	public Pose2D getRobotPose(){
//		return this.robotPose;
//	}
	
	public void setPath2D(Path2D path){
		this.path = path;
	}
	
	public Path2D getPath2D(){
		return this.path;
	}
	
	public void setGoal(double x, double y){
		Point2D p = new Point2D(x, y);
		this.goal = new Pose2D(p, 0);
	}
	
	public void setGoal(Goal goal) {
		setGoal(goal.pose.x, goal.pose.y);
	}
	
	public Pose2D getGoal(){
		return this.goal;
	}
	
	public void onPlan(){
		System.out.println("Start Planning....");
		
		PathPlanParameter param = new PathPlanParameter();
		param.targetPose = this.getGoal();
		param.maxSpeed = new Velocity2D(0.5, 0, 0.5);
		param.distanceTolerance = 9999;
		param.headingTolerance = 9999;
		param.timeLimit = new RTC.Time(10000000, 0);
		
		RTC.Path2D path = rtc.planpath(param);
		if (path != null){
			System.out.println("Path is succesfully acquired.");
			this.setPath2D(path);
		}
	}
	
	public void onStopPlan(){
		this.goal = null;
	}
	
	public void onFollow() {
		System.out.println("Start Following...");
		rtc.followPath(this.getPath2D());
		System.out.println("Following End");
	}
	
}
