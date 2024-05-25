#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_


#include <algorithm>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include <nav_msgs/msg/path.hpp>





using std::vector;
class PlanningVisualization{

private:
    /* data */
    /* visib_pub is seperated from previous ones for different info */
    rclcpp::Node node;
    ros::Publisher goal_pub_;
    ros::Publisher subgoal_pub_;
    ros::Publisher global_path_pub_;
    vector<ros::Publisher> pubs_;

    enum PLANNING_ID {
        GOAL = 1,
        SUBGOAL = 200,
        GLOBAL_PATH = 300,
    };  

public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(rclcpp::Node& nh);

    // draw basic shapes
    void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);

    void displayLineList(const vector<Eigen::Vector3d>& list1,double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);
    
    //draw
    void drawGoal(geometry_msgs::msg::PoseStamped goal, double resolution, const Eigen::Vector4d& color, int id = 0);

    void drawSubgoal(geometry_msgs::msg::PoseStamped subgoal, double resolution,const Eigen::Vector4d& color, int id=0);

    void drawGlobalPath(nav_msgs::msg::Path global_path,double resolution,const Eigen::Vector4d& color, int id=0);

    Eigen::Vector4d getColor(double h, double alpha = 1.0);

    typedef std::shared_ptr<PlanningVisualization> Ptr;
};

#endif