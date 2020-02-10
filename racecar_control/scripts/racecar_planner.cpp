#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <visualization_msgs/MarkerArray.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include "rrtsharp/RRTsharp.h"
#include <queue>
#include "msp/MSP.h"

const int8_t UNKNOWN=255;

ros::Publisher waypoint_pub;
ros::Publisher traj_pub_raw;
ros::Publisher traj_pub_smooth;
ros::Publisher map_pub;

// ====== geometry_msgs is a ROS message used
// ====== to communicate position/orientation
geometry_msgs::PointStamped goal;
geometry_msgs::PointStamped pose;

// ====== State is a defined data structure, declare State<Dimension> variable_name
// ====== variables of type State follow rules of vector addition, norm etc
State<2> startState(0), goalState(0);
geometry_msgs::PointStamped waypoint;

bool planning=false;
bool planned=false;

// To store current path, raw and smoothed
std::deque<State<2>> current_path;//current pose not included
std::deque<State<2>> current_path_raw;//current pose not included

//parameters set in the launch file
double inflation_radius=0.3;
double waypoint_check_distance=0.3;
int nb_obstacle_check=100;
double epsilon=0.1;
double unknownSpaceProbability=0.2;
double waypointMaxDistance=0.1;
bool keepMoving=true;

// ========== Map is stored as a navigation msgs (a partuicular type of ROS msg)
nav_msgs::OccupancyGrid::Ptr local_map;
bool mapChanged=false;

//====================== Find Map Limits =======================
double minX (const nav_msgs::MapMetaData& info)
{
	const geometry_msgs::Polygon p=occupancy_grid_utils::gridPolygon(info);
	return min(p.points[0].x, min(p.points[1].x, min(p.points[2].x, p.points[3].x)));
}

double maxX (const nav_msgs::MapMetaData& info)
{
	const geometry_msgs::Polygon p=occupancy_grid_utils::gridPolygon(info);
	return max(p.points[0].x, max(p.points[1].x, max(p.points[2].x, p.points[3].x)));
}

double minY (const nav_msgs::MapMetaData& info)
{
	const geometry_msgs::Polygon p=occupancy_grid_utils::gridPolygon(info);
	return min(p.points[0].y, min(p.points[1].y, min(p.points[2].y, p.points[3].y)));
}

double maxY (const nav_msgs::MapMetaData& info)
{
	const geometry_msgs::Polygon p=occupancy_grid_utils::gridPolygon(info);
	return max(p.points[0].y, max(p.points[1].y, max(p.points[2].y, p.points[3].y)));
}

//========== Convert geometry_msgs variable to State ===========
State<2> pointState(const geometry_msgs::Point p)
{
	State<2> s;
	s[0]=p.x;
	s[1]=p.y;
	return s;
}
//========== Convert State variable to geometry_msgs ===========
geometry_msgs::Point statePoint(const State<2> s)
{
	geometry_msgs::Point p;
	p.x=s[0];
	p.y=s[1];
	return p;
}
// =============== Publish the computed trajectory to visualize ============
void publishTraj()
{
	visualization_msgs::Marker traj_visu;
	traj_visu.header.frame_id="/map";
	traj_visu.header.stamp=ros::Time::now();
	traj_visu.ns="traj";
	traj_visu.type=visualization_msgs::Marker::LINE_STRIP;
	traj_visu.action=visualization_msgs::Marker::ADD;
	traj_visu.id=1;
	traj_visu.scale.x=0.05;
	traj_visu.scale.y=0.05;
	traj_visu.scale.z=0.05;
	traj_visu.color.r = 1.0;
	traj_visu.color.a = 1.0;

	traj_visu.points.push_back(pose.point);

	for(int i=0;i<current_path.size();++i)
	{
		geometry_msgs::Point p=statePoint(current_path[i]);
		p.z=0.1;
		traj_visu.points.push_back(p);
	}
	traj_pub_smooth.publish(traj_visu);

	traj_visu.points.clear();
	traj_visu.header.stamp=ros::Time::now();
	traj_visu.ns="traj_raw";
	traj_visu.id=2;

	traj_visu.color.r = 0.0;
	traj_visu.color.g = 1.0;

	//traj_visu.points.push_back(pose.point);
	for(int i=0;i<current_path_raw.size();++i)
	{
		geometry_msgs::Point p=statePoint(current_path_raw[i]);
		p.z=0.1;
		traj_visu.points.push_back(p);
	}
	traj_pub_raw.publish(traj_visu);
}
// ======== Send waypoint to be tracked next =================
void sendWaypoint()
{
	waypoint.header.frame_id="/map";
	waypoint.header.stamp=ros::Time::now();
	if(planned){
		waypoint_pub.publish(waypoint);
		publishTraj();
	}
}
// ======== Set waypoint to be tracked next =================
void setWaypoint(State<2> s)
{
	waypoint.point.x=s[0];
	waypoint.point.y=s[1];
	sendWaypoint();
}
// ======= waypoint = current position -> stop =================
void stop()
{
	waypoint=pose;
	sendWaypoint();
}
// ================== Obstacle checking ========================
// ========== Input: State var, Output: boolean ================
bool isObstacle(State<2> state)
{
	if((state-startState).norm()<inflation_radius)
		return false;

  // Convert State-> geometry_msgs
	geometry_msgs::Point point=statePoint(state);
	try
	{
		// Convert geometry_msgs-> map index
		occupancy_grid_utils::index_t index=occupancy_grid_utils::pointIndex(local_map->info,point);
    // Value at the index
		int8_t val=local_map->data[index];
		// Occupancy probabilities are in the range [0,100]. Unknown is -1.
		if(val==UNKNOWN)
		{
			return unknownSpaceProbability>(1-epsilon);
		}
		else
		{
			if(val>100*(1-epsilon))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}
  // If out to bounds -> Obstacle space
	catch(occupancy_grid_utils::CellOutOfBoundsException e)
	{
		return true;
	}
}
// ================== Obstacle probabilities ========================
// ========== Input: State var, Output: float [0,1] ================
double obstacleProbability(State<2> state)
{
	if((state-startState).norm()<inflation_radius)
		return 0.0;
	geometry_msgs::Point point=statePoint(state);
	try
	{
		occupancy_grid_utils::index_t index=occupancy_grid_utils::pointIndex(local_map->info,point);
		int8_t val=local_map->data[index];
		if(val == UNKNOWN)
		{
			return unknownSpaceProbability; //unknown space
		}
		else
		{
			return 0.01*val;
		}
	}
	catch(occupancy_grid_utils::CellOutOfBoundsException e)
	{
		return 1.0;
	}
}
// ================== segment Feasibility ========================
// === Successively check the discretized segment for obstacles===
bool segmentFeasibility(State<2> a,State<2> b)
{
	double res=0.05;
	State<2> inc=b-a;
	double l=inc.norm();
	inc=inc*(res/l);
	for(int i=1;i<l/res;++i)
	{
		if(isObstacle(a+inc*i))
			return false;
	}
	return true;
}
// ================= Smooth trajectories ========================
void smoothTraj()
{
	if(!planned || current_path.size()<2)
		return;
	int i=1;
	State<2> a=startState;
	State<2> b=current_path[i];
	while(i<current_path.size())
	{ // situtation always is of type a->m->b->n
		//if a->b feasible, remove m and set b=n to try to remove b
		if(segmentFeasibility(a,b))
		{
			current_path.erase(current_path.begin()+i-1);
			b=current_path[i];
		}
		else
		{ // if a->b unfeasible, we need to keep a->m, so a becomes m, b becomes n to try to remove the previous b
			a=current_path[i-1];
			i++;
			b=current_path[i];
		}
	}
}
// ========= Create a dense trajecotry, easier for robot to track ============
void densifyWaypoints()
{
	if(waypointMaxDistance>0)
	{
		std::cout << "densifying waypoints" << std::endl;
		int count=0;
		current_path.push_front(startState);
		for(int i=0;i<current_path.size()-1;++i)
		{
			if((current_path[i]-current_path[i+1]).norm()>waypointMaxDistance)
			{
				count++;
				current_path.insert(current_path.begin()+i+1,current_path[i]+(current_path[i+1]-current_path[i])*(waypointMaxDistance/((current_path[i+1]-current_path[i]).norm())));
			}
		}
		current_path.pop_front();
		std::cout << count << " points added" << std::endl;
	}
}
// ================ Check feasibility of the entire calculated path =================
bool checkFeasibility()
{
	//ROS_INFO("checking feasibility");
	if(!planned)
		return true;
	//remove first waypoint if nescessary
	while(sqrt((pose.point.x-waypoint.point.x)*(pose.point.x-waypoint.point.x)+(pose.point.y-waypoint.point.y)*(pose.point.y-waypoint.point.y))<waypoint_check_distance)
	{
		if(current_path.size()>1)
		{
			if(segmentFeasibility(startState,current_path[1]))
			{
				current_path.pop_front();
				setWaypoint(current_path.front());
			}
			else
			{
				//create artificial waypoint so that the tracker keeps sending commands to avoid lock situation
				State<2> s = startState + (current_path.front()-startState)*((float)(waypoint_check_distance+0.2)*(1/(current_path.front()-startState).norm()));
				setWaypoint(s);
				break;
			}
		}
		else
		{
			return true;
		}
	}
	//check feasibility
	//test feasibility from pose to first waypoint
	if(!segmentFeasibility(startState,current_path.front()))
		return false;
	//test feasibility of rest of path
	for(int i=0;i<current_path.size()-1;++i)
	{
		if(!segmentFeasibility(current_path[i],current_path[i+1]))
		return false;
	}
	return true;
}
// ===================== A* Planning =============================

double lambda1=0.0;

struct PQItem
{
  occupancy_grid_utils::index_t ind; // index of current cell
  double g_cost; // cost-to-come from start
  double h_cost; // heuristic estimate of cost-to-go
  occupancy_grid_utils::index_t parent_ind; // index of the parent cell
	// constructor
  PQItem (occupancy_grid_utils::index_t ind, double g_cost, double h_cost, occupancy_grid_utils::index_t parent_ind) :
    ind(ind), g_cost(g_cost), h_cost(h_cost), parent_ind(parent_ind) {}
	// operator to comapre two PQItems: i1 (current) and i2
	// This will be used by the std::priority_queue to order the open list
  bool operator < (const PQItem& i2) const
  {
    // i2 would be preferred over i1 if f value of i1 > f value of i2
    return ((g_cost + h_cost) > (i2.g_cost + i2.h_cost));
  }
};

geometry_msgs::Point indexPoint(occupancy_grid_utils::index_t ind)
{
	return occupancy_grid_utils::cellCenter(local_map->info,occupancy_grid_utils::indexCell(local_map->info,ind));
}

State<2> indexState(occupancy_grid_utils::index_t ind)
{
	return pointState(indexPoint(ind));
}

double g(occupancy_grid_utils::index_t ind)
{
	double resolution = local_map->info.resolution;
	// return resolution*(1.0-lambda1)+lambda1*obstacleProbability(indexState(ind));
	return resolution*(1.0-lambda1);
}

double h(occupancy_grid_utils::index_t ind)
{
	return 1*(1.0-lambda1)*(goalState-indexState(ind)).norm();
}

void Astar_planning()
{
	ROS_INFO("A star planning");
	// Initialize a open_list (priority_queue) to keep track of unexpanded nodes
	std::priority_queue<PQItem> open_list;
	// total number of cells in the grid
	const unsigned num_cells = local_map->info.height*local_map->info.width;
	// initialize a closed list to keep track of visited nodes
	std::vector<bool> closed_list(num_cells); // Default initialized to all false
  // Start and Goal cell index
	const occupancy_grid_utils::index_t dest_ind = occupancy_grid_utils::pointIndex(local_map->info,goal.point);
	const occupancy_grid_utils::index_t src_ind = occupancy_grid_utils::pointIndex(local_map->info,statePoint(startState));
  // Push the start graph node into the open list to begin the search
	open_list.push(PQItem(src_ind, 0, h(src_ind),src_ind));
  // Initialize an vector array to store the parent index for very cell
	// This will be used for back tracking the path once goal vertex is expanded
	std::vector<occupancy_grid_utils::index_t> parent(num_cells,0);
	// Begin the search
	while (!open_list.empty())
	{
    // pop the best (top) node from open_list and store it in a var of type PQItem
    const PQItem current = open_list.top();
    open_list.pop();
		// Is the current node already expanded ?
    if (closed_list[current.ind])
      continue; // if yes pop next one
    // store the parent of current node in the parent array
    parent[current.ind] = current.parent_ind;
    // Make sure that current node is added to closed_list
    closed_list[current.ind] = true;
    // Check if current node is the goal node
    if (current.ind == dest_ind)
		{
			std::cout << "solution found" << std::endl;
			std::cout << "Visited " << std::count(closed_list.begin(), closed_list.end(), true) << " states out of " << num_cells << std::endl;;
			std::deque<occupancy_grid_utils::index_t> path;
			path.push_back(dest_ind);
			occupancy_grid_utils::index_t last = dest_ind;
			while (parent[last]!=src_ind)
			{
				path.push_front(parent[last]);
				last=parent[last];
			}

			current_path_raw.resize(path.size());
			std::transform(path.begin(), path.end(), current_path_raw.begin(), &indexState);
			planned=true;
			return;
    }
    // Current node is not the goal, then expand it: Add its neighbors to open_list
		// Get the cell Representation of the current node
		const occupancy_grid_utils::Cell c = occupancy_grid_utils::indexCell(local_map->info, current.ind);
    // Implement a loop to get all and store all the neighboring cells
		for (int d=-1; d<=1; d+=2)
		{
			for (int vertical=0; vertical<2; vertical++)
			{
				const int cx = c.x + d*(1-vertical);
				const int cy = c.y + d*vertical;
				if (cx>=0 && cy>=0)
				{
					const occupancy_grid_utils::Cell c2((occupancy_grid_utils::coord_t) cx, (occupancy_grid_utils::coord_t) cy);
					if (withinBounds(local_map->info, c2))
					{
						const occupancy_grid_utils::index_t ind = cellIndex(local_map->info, c2);
						if (!isObstacle(pointState(indexPoint(ind))) && !closed_list[ind])
						{
							open_list.push(PQItem(ind, current.g_cost + g(ind), h(ind), current.ind));
						}
					}
				}
			}
    }
  }
	// If no item in the open_list and still have not found a path then
	ROS_INFO("Planning failed");
	planning=false;
}

//============== OMPL based code for Sampling-based planners ============
namespace ob = ompl::base;
namespace og = ompl::geometric;
//======== Convert OMPL State to user defined State var ======
State<2> obstateState(const ob::State* s){
	State<2> s2;
	s2[0]=s->as<ob::RealVectorStateSpace::StateType>()->values[0];
	s2[1]=s->as<ob::RealVectorStateSpace::StateType>()->values[1];
	return s2;
}
//======== Define a objective function =============
//======== This is a state cost integral objective that penalizes states with ==
//======== high obstacle probabilities  ========================================
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }
    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(1.0-lambda1+lambda1*obstacleProbability(obstateState(s)));
    }
};
// =========== Obstacle Checking function for OMPL planners ===================
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}

    bool isValid(const ob::State* s) const
    {
        return !isObstacle(obstateState(s));
    }
};
//========= RRT based planners ==========
void RRT_planning(){
    ROS_INFO("RRT star planning");
		// initialize a vector search space
		ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
		// set bounds on those vector search space
    space->as<ob::RealVectorStateSpace>()->setBounds(minX(local_map->info), maxX(local_map->info));
		// Longest segment in the graph
		space->setLongestValidSegmentFraction(0.01/(maxX(local_map->info)-minX(local_map->info)));
		// pointer to access space information
		ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
		// set up obstacle checker
		si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setup(); //setup state space
		//Set start and goal
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = startState[0];
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = startState[1];
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goalState[0];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goalState[1];
		// Problem definition pointer
		ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
		pdef->setStartAndGoalStates(start, goal);
    // pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ClearanceObjective(si)));
    ompl::base::OptimizationObjectivePtr opt = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    pdef->setOptimizationObjective(opt); // set optimization objective
		// Now select planning algorithm
    og::InformedRRTstar *plan_pt=new og::InformedRRTstar(si);
		//og::RRTsharp *plan_pt=new og::RRTsharp(si);
		// auto plan_pt(std::make_shared<og::RRTsharp>(si));
		// Set Planner parameters
    plan_pt->setGoalBias(0.1);
    plan_pt->setRange(1.);
		plan_pt->setInformedSampling(true);
		plan_pt->setTreePruning(true);
    ob::PlannerPtr optimizingPlanner(plan_pt);
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();
    ob::PlannerStatus solved;
    int it=0;
    while(solved!=ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION && it<1)
		{
			it++;
			// Give some time for planner to solve the Problem
			solved = optimizingPlanner->solve(1.0);
    }
		// If solved, then extract the solution
    if(solved==ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION)
		{
			ob::PathPtr path =pdef->getSolutionPath();
			og::PathGeometric geo_path = static_cast<ompl::geometric::PathGeometric&>(*path);

			std::vector<ob::State *> sol = geo_path.getStates();
			std::cout << "sol length" <<sol.size()<< '\n';
			// std::vector< ob::State * > sol = boost::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->getStates();
			current_path_raw.resize(sol.size());
			std::transform(sol.begin(), sol.end(), current_path_raw.begin(), &obstateState);
			planned=true;
		}
		else
		{
			ROS_INFO("Planning failed");
			planned=false;
    }
}

void plan(){
	ROS_INFO("planning");
	ROS_INFO_STREAM("pose: "<< startState[0] << "," << startState[1]);

	if(keepMoving)
	{
		if(planned && current_path.size()>0)
		{
			//look for farther free space straighline of previous trajectory
			int i=0;
			while(i<current_path.size() && segmentFeasibility(startState,current_path[i]))
			{
				++i;
			}
			--i;
			State<2> s=startState+(current_path[i]-startState)*std::min((current_path[i]-startState).norm(),1.0f)*(1/(current_path[i]-startState).norm());
			setWaypoint(s); //set and send the waypoint
			startState=s; //plan from that waypoint
		}
	}
	else
	{
		stop();
	}
	planned=false;
	planning=true;

  Astar_planning();

	if(planned)
	{
		current_path=std::deque<State<2>>();
		current_path.assign(current_path_raw.begin(),current_path_raw.end());
		std::cout << "smoothed solution" <<std::endl;
		//for(int i=0;i<current_path_raw.size();++i)
		for(int i=0;i<1;++i)
			smoothTraj();
		if(waypointMaxDistance>0)
			densifyWaypoints();

		setWaypoint(current_path.front());
	}
	planning=false;
}

// ======= Upadate position of robot from SLAM/ localization package ==========
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	// ROS_INFO_STREAM("pose update: "<< msg->pose.position.x << "," << msg->pose.position.y);
	if(planning)
		return;
	//update local pose
	pose.header.frame_id=msg->header.frame_id;
	pose.header.stamp=ros::Time(0);
	pose.point.x=msg->pose.position.x;
	pose.point.y=msg->pose.position.y;

	startState[0]=pose.point.x;
	startState[1]=pose.point.y;

	if(!checkFeasibility()){
		plan();
	}
	sendWaypoint();
}
// ======= Upadate map from SLAM/map_Server package ==========
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//ROS_INFO("map update");
	if(planning)
		return;
	//inflate update local map
	local_map=occupancy_grid_utils::inflateObstacles(*msg,inflation_radius,true);
	map_pub.publish(local_map);
	mapChanged=true;
	if(!checkFeasibility())
	{
		plan();
	}
	sendWaypoint();
}
// ========= If the user changes the goal position =====================
void goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	if(planning)
		return;
	//update local goal
	goal.header.frame_id=msg->header.frame_id;
	goal.header.stamp=ros::Time(0);
	goal.point.x=msg->point.x;
	goal.point.y=msg->point.y;

	if((goalState[0]!=goal.point.x)&&(goalState[1]!=goal.point.y))
	{
		ROS_INFO_STREAM("goal update: "<< msg->point.x << "," << msg->point.y);
		goalState[0]=goal.point.x;
		goalState[1]=goal.point.y;
		plan();
	}
	sendWaypoint();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "racecar_planner8803"); //node name
  ros::NodeHandle n;
  ros::NodeHandle nh_rel=ros::NodeHandle("~");
  //reading parameters
  nh_rel.param("nb_obstacle_check",nb_obstacle_check,100);
  nh_rel.param("epsilon",epsilon,0.1);
  nh_rel.param("inflation_radius",inflation_radius,0.1);
  nh_rel.param("waypoint_check_distance",waypoint_check_distance,0.3);
  nh_rel.param("unknown_space_probability",unknownSpaceProbability,0.5);
  nh_rel.param("waypoint_max_distance",waypointMaxDistance,0.1);
  nh_rel.param("keep_moving",keepMoving,true);
  int queue_size=5;
  // Publish the following information
  waypoint_pub = n.advertise<geometry_msgs::PointStamped>("/waypoint", queue_size); //Next position togo
  traj_pub_raw = n.advertise<visualization_msgs::Marker>("/traj_raw", queue_size); // raw path
  traj_pub_smooth = n.advertise<visualization_msgs::Marker>("/traj_smooth", queue_size); // smoothed path
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_inflated", queue_size); // inflated map
	// Take as input (subscribe) following information
  ros::Subscriber goal_sub = n.subscribe("/goal_pose", queue_size, goalCallback); // user set goal position
  ros::Subscriber pose_sub = n.subscribe("/slam_out_pose", queue_size, poseCallback);// current position
  ros::Subscriber map_sub = n.subscribe("/map", queue_size, mapCallback); // Map from lidar/map server

  ros::spin();
  return 0;
}
