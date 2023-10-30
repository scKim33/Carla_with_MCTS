#include <boost/thread/thread.hpp>

#include "../include/mcts_random.h"
#include "../include/utils.h"


using namespace std;


// ROS Nodes
ros::Publisher Pub_StartPose, Pub_GoalPose, Pub_CollisionCheck, Pub_MctsPath, Pub_Action, Pub_CarPose;
ros::Subscriber Sub_GoalPosition, Sub_LocalizationData, Sub_ObstaclePoses, Sub_Manual_Goal;
geometry_msgs::PoseStamped poseStamped;
geometry_msgs::PoseArray collision_edge;
std_msgs::Header m_header;


// State Variables
Coordinate goal;
Coordinate car;
State state;
vector<vector<double>> obstacle_poses;


// Util Variables
int restart_scenario = 0;
bool Manual_Goal = false;
bool state_initialize = true;


// Visualizing car
double m_collision_chk_len = 1.40;
double m_safe_width = 1.16;
double m_point_safe_region1 = 0.27;
double m_point_safe_region2 = 0.27;
double m_side_safe_region = 0.27;

template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{

  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
    {
      linspaced.push_back(start);
      return linspaced;
    }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
  return linspaced;
}

void collision_range(double x, double y, double theta) { // vehicle collision check range
    collision_edge.header = m_header;        
    std::vector<double> angle_set = linspace(0.0, 2.0 * M_PI, 20);
    geometry_msgs::PoseStamped _poseStamped;
    // Large Circle
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_safe_width * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_safe_width * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    // Large Circle1
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_collision_chk_len*cos(theta) + m_safe_width * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_collision_chk_len*sin(theta) + m_safe_width * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    // Large Circle2
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + 2.0 * m_collision_chk_len*cos(theta) + m_safe_width * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + 2.0 * m_collision_chk_len*sin(theta) + m_safe_width * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }        

    // point circle
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x - 0.719*cos(theta) - 0.9175*sin(theta) + m_point_safe_region1 * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y - 0.719*sin(theta) + 0.9175*cos(theta) + m_point_safe_region1 * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x - 0.719*cos(theta) + 0.9175*sin(theta) + m_point_safe_region1 * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y - 0.719*sin(theta) - 0.9175*cos(theta) + m_point_safe_region1 * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + 2*m_collision_chk_len*cos(theta) + 0.735*cos(theta) - 0.9175*sin(theta) + m_point_safe_region2 * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + 2*m_collision_chk_len*sin(theta) + 0.735*sin(theta) + 0.9175*cos(theta) + m_point_safe_region2 * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + 2*m_collision_chk_len*cos(theta) + 0.735*cos(theta) + 0.9175*sin(theta) + m_point_safe_region2 * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + 2*m_collision_chk_len*sin(theta) + 0.735*sin(theta) - 0.9175*cos(theta) + m_point_safe_region2 * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }

    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_collision_chk_len*cos(theta) - 0.6593*cos(theta) - 0.8587*sin(theta) + m_side_safe_region * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_collision_chk_len*sin(theta) - 0.6593*sin(theta) + 0.8587*cos(theta) + m_side_safe_region * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_collision_chk_len*cos(theta) - 0.6593*cos(theta) + 0.8587*sin(theta) + m_side_safe_region * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_collision_chk_len*sin(theta) - 0.6593*sin(theta) - 0.8587*cos(theta) + m_side_safe_region * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_collision_chk_len*cos(theta) + 0.6593*cos(theta) - 0.8587*sin(theta) + m_side_safe_region * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_collision_chk_len*sin(theta) + 0.6593*sin(theta) + 0.8587*cos(theta) + m_side_safe_region * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_collision_chk_len*cos(theta) + 0.6593*cos(theta) + 0.8587*sin(theta) + m_side_safe_region * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_collision_chk_len*sin(theta) + 0.6593*sin(theta) - 0.8587*cos(theta) + m_side_safe_region * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    // if (VISUALIZATION)
    Pub_CollisionCheck.publish(collision_edge);
    collision_edge.poses.clear();
}


void Callback_Goal(const geometry_msgs::PoseArray::ConstPtr& end) {
    m_header.stamp = ros::Time::now();
    m_header.frame_id = "map";

    goal.th = tf::getYaw(end->poses[0].orientation);
    goal.x = end->poses[0].position.x - 1.2865250458893225 * cos(goal.th);//; + map_range/2.0;
    goal.y = end->poses[0].position.y - 1.2865250458893225 * sin(goal.th);//+ map_range/2.0;

    poseStamped.pose.position.x = goal.x;
    poseStamped.pose.position.y = goal.y;
    poseStamped.pose.position.z = 1.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(goal.th);
    poseStamped.header = m_header;
    poseStamped.pose.orientation = odom_quat;
    Pub_GoalPose.publish(poseStamped);

    poseStamped.pose.position.x = -1.2865250458893225;
    poseStamped.pose.position.y = 0.0;
    poseStamped.pose.position.z = 1.0;
    geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(0.0);
    poseStamped.header = m_header;
    poseStamped.pose.orientation = odom_quat2;
    Pub_StartPose.publish(poseStamped);
}


void Callback_LocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg)  {
    car.x = msg->data.at(0) - 1.2865250458893225 * cos(msg->data.at(2));
    car.y = msg->data.at(1) - 1.2865250458893225 * sin(msg->data.at(2));
    car.th = msg->data.at(2);
    
    m_header.stamp = ros::Time::now();
    m_header.frame_id = "map";
    poseStamped.pose.position.x = car.x;
    poseStamped.pose.position.y = car.y;
    poseStamped.pose.position.z = 1.0;
    geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(car.th);
    poseStamped.header = m_header;
    poseStamped.pose.orientation = odom_quat2;
    Pub_CarPose.publish(poseStamped);


    collision_range(car.x, car.y, car.th);
}


void Callback_Restart(const std_msgs::Int32::ConstPtr& msg) {
    restart_scenario = msg->data;
}


void Callback_ManualGoal(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    state_initialize = true;
}


void Thread_ROS() {
    int argc = 0;
    char** argv;
    ros::init(argc, argv, "ROS_node");
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh("~");
    ros::CallbackQueue q;
    ros::NodeHandle nh_q;
    nh_q.setCallbackQueue(&q);

    // Asyncspinner is used to separate callback function thread
    ros::AsyncSpinner spinner(0, &q);
    spinner.start();

    nh_.getParam("Manual_Goal", Manual_Goal);

    Sub_GoalPosition = nh_.subscribe("/parking_cands", 1, &Callback_Goal);
    Sub_LocalizationData = nh_.subscribe("/LocalizationData", 1, &Callback_LocalizationData);
    Sub_Restart = nh_.subscribe("restart_flag", 1, &Callback_Restart);
    Sub_Manual_Goal = nh_.subscribe("/move_base_simple/goal", 1, &Callback_ManualGoal);
    Pub_CarPose = nh_.advertise<geometry_msgs::PoseStamped>("PoseCar", 1);
    Pub_StartPose = nh_.advertise<geometry_msgs::PoseStamped>("PoseStart", 1);
    Pub_GoalPose = nh_.advertise<geometry_msgs::PoseStamped>("PoseGoal", 1);
    Pub_CollisionCheck = nh_.advertise<geometry_msgs::PoseArray>("collision_edge", 1);
    Pub_MctsPath = nh_.advertise<nav_msgs::Path>("mcts_path_opt", 1);
    Pub_Action =  nh_.advertise<std_msgs::Float32MultiArray>("Control_Command", 1);

    ros::spin();
}

void Thread_Observation() {
    int argc = 0;
    char** argv;
    ros::init(argc, argv, "Observation_node");
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh("~");

    ros::Rate r(40);
    while(ros::ok()) {
        // state.pos = car;

        ros::spinOnce();
        r.sleep();
    }
}


void Thread_MCTS() {
    int argc = 0;
    char** argv;
    ros::init(argc, argv, "MCTS_node");
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh("~");

    // Takes time to get ROS data
    sleep(3);

    // Define ROS variables
    nav_msgs::Path mcts_path;
    std_msgs::Float32MultiArray msg;

    ros::Rate r(20);
    while(ros::ok()) {
        // get car position from carla
        state.pos = car;

        // Terminate condition
        if(!Manual_Goal && (state.pos.x, state.pos.y, goal.x, goal.y) < 0.5) {
            cout << "Terminate scenario as the car reaches the goal nearby.\nx error: " << state.pos.x - goal.x << ", y error: " << state.pos.y - goal.y << "\n";
            msg.data.clear();
            msg.data.push_back(0.0);
            msg.data.push_back(0.0); // deg, km/h
            PubAction.publish(msg);
            break;
        }

        // If we manually set the new goal, steering and velocity is set to zero for better MCTS simulation.
        if(state_initialize) {
            state.s = 0;
            state.v = 0;
            state_initialize = false;
        }

        // Initialize mcts agent
        MCTS mcts_agent(nh_, state, goal);

        // MCTS main function gives the next state and mostly visited MCTS path
        tie(state.s, state.v, mcts_path) = mcts_agent.main();

        // Publish control command and mcts path
        msg.data.clear();
        msg.data.push_back(state.s * RAD2DEG);
        msg.data.push_back(state.v * MPS2KMPH); // deg, km/h
        PubAction.publish(msg);
        Pub_MctsPath.publish(mcts_path);

        ros::spinOnce();
        r.sleep();
    }

}

int main(int argc, char* argv[])
{
    try {
        boost::thread t1 = boost::thread(boost::bind(&Thread_ROS));
        boost::thread t2 = boost::thread(boost::bind(&Thread_Observation));
        boost::thread t3 = boost::thread(boost::bind(&Thread_MCTS));
        t1.join();
        t2.join();
        t3.join();
    }
    catch(std::exception& e) {
        std::cerr << "------------ error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "------------ Exception of unknown type!\n";
    }

    return 0;
} 