// -*- Java -*-
/*!
 * @file  StringPathPlannerImpl.java
 * @brief String Path Planner 
 * @date  $Date$
 *
 * $Id$
 */

import goaldata.UnformattedException;
import goaldata.YAMLReader;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Calendar;

import RTC.OGMap;
import RTC.OGMapServer;
import RTC.OGMapHolder;
import RTC.Path2D;
import RTC.Path2DHolder;
import RTC.PathPlanner;
import RTC.PathFollower;
import RTC.PathPlanParameter;
import RTC.Point2D;
import RTC.Pose2D;
import RTC.RETURN_VALUE;
import RTC.ReturnCode_t;
import RTC.Time;
import RTC.TimedString;
import RTC.TimedPose2D;
import RTC.RangeData;
import RTC.TimedVelocity2D;
import RTC.Waypoint2D;
import jp.go.aist.rtm.RTC.DataFlowComponentBase;
import jp.go.aist.rtm.RTC.Manager;
import jp.go.aist.rtm.RTC.port.InPort;
import jp.go.aist.rtm.RTC.port.OutPort;
import jp.go.aist.rtm.RTC.util.DataRef;
import jp.go.aist.rtm.RTC.port.CorbaConsumer;
import jp.go.aist.rtm.RTC.port.CorbaPort;


/*!
 * @class StringPathPlannerImpl
 * @brief String Path Planner 
 *
 */
public class StringPathPlannerImpl extends DataFlowComponentBase {

	private StringPathPlannerFunc func;
	private CommandManager manager;
	private YAMLReader yaml;

	private Calendar m_lastReceivedTime;
	private float m_poseTimeout = (float) 3.0;

	/*
	 * !
	 * 
	 * @brief constructor
	 * 
	 * @param manager Maneger Object
	 */
	public StringPathPlannerImpl(Manager manager) {
		super(manager);
		// <rtc-template block="initializer">
		m_command_val = new TimedString();
		m_command = new DataRef<TimedString>(m_command_val);
		m_commandIn = new InPort<TimedString>("command", m_command);
		m_currentPose_val = new TimedPose2D(new Time(0, 0), new Pose2D(
				new Point2D(0, 0), 0));
		m_currentPose = new DataRef<TimedPose2D>(m_currentPose_val);
		m_currentPoseIn = new InPort<TimedPose2D>("currentPose", m_currentPose);
		m_range_val = new RangeData();
		m_range = new DataRef<RangeData>(m_range_val);
		m_rangeIn = new InPort<RangeData>("range", m_range);
		m_targetVelocity_val = new TimedVelocity2D(new RTC.Time(0, 0),
				new RTC.Velocity2D(0, 0, 0));
		m_targetVelocity = new DataRef<TimedVelocity2D>(m_targetVelocity_val);
		m_targetVelocityOut = new OutPort<TimedVelocity2D>("targetVelocity",
				m_targetVelocity);
		m_goalPoint_val = new Waypoint2D();
		m_goalPoint = new DataRef<Waypoint2D>(m_goalPoint_val);
		m_goalPointOut = new OutPort<Waypoint2D>("goalPoint", m_goalPoint);
		m_mapServerPort = new CorbaPort("mapServer");
		m_pathPlannerPort = new CorbaPort("pathPlanner");
		m_pathFollowerPort = new CorbaPort("pathFollower");
		// </rtc-template>

	}

	/**
	 * 
	 * The initialize action (on CREATED->ALIVE transition) formaer
	 * rtc_init_entry()
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	@Override
	protected ReturnCode_t onInitialize() {
		// Registration: InPort/OutPort/Service
		// <rtc-template block="registration">
		// Set InPort buffers
		addInPort("command", m_commandIn);
		addInPort("currentPose", m_currentPoseIn);
		addInPort("range", m_rangeIn);

		// Set OutPort buffer
		addOutPort("targetVelocity", m_targetVelocityOut);
		addOutPort("goalPoint", m_goalPointOut);

		// Set service consumers to Ports
		m_mapServerPort.registerConsumer("mapServer", "RTC::OGMapServer",
				m_OGMapServerBase);
		m_pathPlannerPort.registerConsumer("PathPlanner", "RTC::PathPlanner",
				m_pathPlannerBase);
		m_pathFollowerPort.registerConsumer("PathFollower",
				"RTC::PathFollower", m_pathFollowerBase);

		// Set CORBA Service Ports
		addPort(m_mapServerPort);
		addPort(m_pathPlannerPort);
		addPort(m_pathFollowerPort);

		// </rtc-template>

		try {
			this.yaml = new YAMLReader(new File("coordinates.yaml"));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return RTC.ReturnCode_t.RTC_ERROR;
		} catch (UnformattedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return RTC.ReturnCode_t.RTC_ERROR;			
		}
		this.func = new StringPathPlannerFunc(this);
		this.manager = new CommandManager(func, yaml);

		return super.onInitialize();
	}

	/***
	 * 
	 * The finalize action (on ALIVE->END transition) formaer
	 * rtc_exiting_entry()
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	// @Override
	// protected ReturnCode_t onFinalize() {
	// return super.onFinalize();
	// }

	/***
	 * 
	 * The startup action when ExecutionContext startup former
	 * rtc_starting_entry()
	 * 
	 * @param ec_id
	 *            target ExecutionContext Id
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	// @Override
	// protected ReturnCode_t onStartup(int ec_id) {
	// return super.onStartup(ec_id);
	// }

	/***
	 * 
	 * The shutdown action when ExecutionContext stop former
	 * rtc_stopping_entry()
	 * 
	 * @param ec_id
	 *            target ExecutionContext Id
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	// @Override
	// protected ReturnCode_t onShutdown(int ec_id) {
	// return super.onShutdown(ec_id);
	// }

	/***
	 * 
	 * The activated action (Active state entry action) former
	 * rtc_active_entry()
	 * 
	 * @param ec_id
	 *            target ExecutionContext Id
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	@Override
	protected ReturnCode_t onActivated(int ec_id) {
		m_lastReceivedTime = Calendar.getInstance();

		System.out.println("Activated");
		return super.onActivated(ec_id);
	}

	/***
	 * 
	 * The deactivated action (Active state exit action) former
	 * rtc_active_exit()
	 * 
	 * @param ec_id
	 *            target ExecutionContext Id
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	@Override
	protected ReturnCode_t onDeactivated(int ec_id) {
		System.out.println("Deactivated");
		return super.onDeactivated(ec_id);
	}

	/***
	 * 
	 * The execution action that is invoked periodically former rtc_active_do()
	 * 
	 * @param ec_id
	 *            target ExecutionContext Id
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	@Override
	protected ReturnCode_t onExecute(int ec_id) {
		Calendar currentTime = Calendar.getInstance();
		double duration = currentTime.getTimeInMillis()
				- m_lastReceivedTime.getTimeInMillis();

		if (m_currentPoseIn.isNew()) {
			m_currentPoseIn.read();
			this.func.setRobotPose(m_currentPose.v.data);
		}

		if (m_commandIn.isNew()) {
			m_commandIn.read();

			String command = m_command.v.data;
			System.out.println("[command read] " + command);

			try {
				if (duration > m_poseTimeout * 1000 && m_poseTimeout > 0) {
					manager.doCommand(command);
					m_lastReceivedTime = currentTime;
				} else {
					System.out.println("Busy");
				}
			} catch (InvalidCommandException e) {
				e.printStackTrace();
				return RTC.ReturnCode_t.RTC_ERROR;
			}
		}

		return super.onExecute(ec_id);
	}

	/***
	 * 
	 * The aborting action when main logic error occurred. former
	 * rtc_aborting_entry()
	 * 
	 * @param ec_id
	 *            target ExecutionContext Id
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	// @Override
	// public ReturnCode_t onAborting(int ec_id) {
	// return super.onAborting(ec_id);
	// }

	/***
	 * 
	 * The error action in ERROR state former rtc_error_do()
	 * 
	 * @param ec_id
	 *            target ExecutionContext Id
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	// @Override
	// public ReturnCode_t onError(int ec_id) {
	// return super.onError(ec_id);
	// }

	/***
	 * 
	 * The reset action that is invoked resetting This is same but different the
	 * former rtc_init_entry()
	 * 
	 * @param ec_id
	 *            target ExecutionContext Id
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	@Override
	protected ReturnCode_t onReset(int ec_id) {
		return super.onReset(ec_id);
	}

	/***
	 * 
	 * The state update action that is invoked after onExecute() action no
	 * corresponding operation exists in OpenRTm-aist-0.2.0
	 * 
	 * @param ec_id
	 *            target ExecutionContext Id
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	// @Override
	// protected ReturnCode_t onStateUpdate(int ec_id) {
	// return super.onStateUpdate(ec_id);
	// }

	/***
	 * 
	 * The action that is invoked when execution context's rate is changed no
	 * corresponding operation exists in OpenRTm-aist-0.2.0
	 * 
	 * @param ec_id
	 *            target ExecutionContext Id
	 * 
	 * @return RTC::ReturnCode_t
	 * 
	 * 
	 */
	// @Override
	// protected ReturnCode_t onRateChanged(int ec_id) {
	// return super.onRateChanged(ec_id);
	// }
	//
	// DataInPort declaration
	// <rtc-template block="inport_declare">
	protected TimedString m_command_val;
	protected DataRef<TimedString> m_command;
	/*
	 * !
	 */
	protected InPort<TimedString> m_commandIn;

	protected TimedPose2D m_currentPose_val;
	protected DataRef<TimedPose2D> m_currentPose;
	/*
	 * !
	 */
	protected InPort<TimedPose2D> m_currentPoseIn;

	protected RangeData m_range_val;
	protected DataRef<RangeData> m_range;
	/*
	 * !
	 */
	protected InPort<RangeData> m_rangeIn;

	// </rtc-template>

	// DataOutPort declaration
	// <rtc-template block="outport_declare">
	protected TimedVelocity2D m_targetVelocity_val;
	protected DataRef<TimedVelocity2D> m_targetVelocity;
	/*
	 * !
	 */
	protected OutPort<TimedVelocity2D> m_targetVelocityOut;

	protected Waypoint2D m_goalPoint_val;
	protected DataRef<Waypoint2D> m_goalPoint;
	/*
	 * !
	 */
	protected OutPort<Waypoint2D> m_goalPointOut;

	// </rtc-template>

	// CORBA Port declaration
	// <rtc-template block="corbaport_declare">
	/*
	 * !
	 */
	protected CorbaPort m_mapServerPort;
	/*
	 * !
	 */
	protected CorbaPort m_pathPlannerPort;
	/*
	 * !
	 */
	protected CorbaPort m_pathFollowerPort;

	// </rtc-template>

	// Service declaration
	// <rtc-template block="service_declare">

	// </rtc-template>

	// Consumer declaration
	// <rtc-template block="consumer_declare">
	protected CorbaConsumer<OGMapServer> m_OGMapServerBase = new CorbaConsumer<OGMapServer>(
			OGMapServer.class);
	/*
	 * !
	 */
	protected OGMapServer m_OGMapServer;
	protected CorbaConsumer<PathPlanner> m_pathPlannerBase = new CorbaConsumer<PathPlanner>(
			PathPlanner.class);
	/*
	 * !
	 */
	protected PathPlanner m_pathPlanner;
	protected CorbaConsumer<PathFollower> m_pathFollowerBase = new CorbaConsumer<PathFollower>(
			PathFollower.class);
	/*
	 * !
	 */
	protected PathFollower m_pathFollower;

	// </rtc-template>

	public OGMap requestMap() {

		OGMap map = new OGMap();
		OGMapHolder mapHolder = new OGMapHolder(map);

		if (this.m_mapServerPort.get_connector_profiles().length != 0) {
			RETURN_VALUE retval;
			retval = this.m_OGMapServerBase._ptr().requestCurrentBuiltMap(
					mapHolder);

			if (retval == RETURN_VALUE.RETVAL_OK) {
				return mapHolder.value;
			} else if (retval == RETURN_VALUE.RETVAL_EMPTY_MAP) {
				System.out.println("ERROR: Empty Map");
			}
		}
		return null;
	}

	public Path2D planpath(PathPlanParameter param) {

		Path2DHolder pathHolder = new Path2DHolder();

		param.currentPose = new RTC.Pose2D(new RTC.Point2D(
				this.m_currentPose.v.data.position.x,
				this.m_currentPose.v.data.position.y), 0);
		param.map = requestMap();

		if (m_pathPlannerPort.get_connector_profiles().length != 0) {
			RETURN_VALUE retval;
			retval = this.m_pathPlannerBase._ptr().planPath(param, pathHolder);
			if (retval == RETURN_VALUE.RETVAL_OK) {
				System.out.println("SUCCESS: Planning Success");
			} else if (retval == RETURN_VALUE.RETVAL_NOT_FOUND) {
				System.out.println("ERROR: Path Not Found");
			} else if (retval == RETURN_VALUE.RETVAL_INVALID_PARAMETER) {
				System.out.println("ERROR: Invalid Start or Goal coordinates");
			}
		}

		return pathHolder.value;
	}

	Path2D followingTargetPath;
	Thread followingTask;

	public void followPath(Path2D path) {
		followingTargetPath = path;

		followingTask = new Thread(new Runnable() {
			public void run() {
				follow();
			}
		});

		followingTask.start();
	}

	public void follow() {
		Path2D path = followingTargetPath;

		if (m_pathFollowerPort.get_connector_profiles().length != 0) {
			RETURN_VALUE retval;
			retval = this.m_pathFollowerBase._ptr().followPath(path);
			if (retval == RETURN_VALUE.RETVAL_OK) {
				System.out.println("SUCCESS: FOLLOW SUCCESS");
				return;
			} else if (retval == RETURN_VALUE.RETVAL_EMERGENCY_STOP) {
				System.out.println("ERROR: FOLLOWING EMERGENCY STOP");
				return;
			} else if (retval == RETURN_VALUE.RETVAL_CURRENT_POSE_TIME_OUT) {
				System.out
						.println("ERROR: FOLLOWING Localization disconnected or Kobuki error");
				return;
			} else if (retval == RETURN_VALUE.RETVAL_CURRENT_POSE_INVALID_VALUE) {
				System.out
						.println("ERROR: FOLLOWING Localization sent Strange Value");
				return;
			} else {
				System.out
						.println("ERROR: FOLLOWING FAILED with UNKNOWN ERROR");
				return;
			}
		}
	}

}
