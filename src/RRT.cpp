#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "RRT.h"
#include "Grid_map.h"

// Rviz visualization
void initializeRviz(ros::Publisher marker_pub, const RRT& rrt);
void populateObstacles(ros::Publisher marker_pub, const RRT& rrt);
void addEdge(Node* from, Node* to, ros::Publisher marker_pub, bool success);
void drawFinalPath(Graph graph, ros::Publisher marker_pub);
static bool done_path_drawing = false;
static bool initialization = false;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RRT");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    int sampling_cnt = 0;


    // initialize RRT
    // rrt(start_x, start_y, target_x, target_y, x_range, y_range)
    RRT rrt(0, 0, 350, 350, 400, 400);
    Node* root_node = new Node(rrt.start_x, rrt.start_y);
    root_node->parent = nullptr;
    rrt.graph.add_node(root_node);

    while(ros::ok()){
        // add marker in Rviz
        while (marker_pub.getNumSubscribers() < 1){
            if (!ros::ok()){
                return 0;
            }
            ROS_WARN_ONCE("Please run Rviz in another terminal");
            sleep(1);
        }

//        if (!initialization){
//        }
        initializeRviz(marker_pub, rrt);

        // Rviz visualization
        if (!rrt.success && sampling_cnt < rrt.num_sampling) {
            ROS_INFO("iteration: %d", sampling_cnt);
            // random node coordinate generation
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(0, 400 - 1);
            int rand_x = dis(gen);
            int rand_y = dis(gen);

            Node rand_node = Node(rand_x, rand_y);

            // find nearest node from random node
            double nearest_distance = std::numeric_limits<double>::infinity();
            Node* nearest_node;
            for (const auto& candidate_node: rrt.graph.get_node_list()) {
                double candidate_distance = rand_node.get_distance_to(candidate_node);
                if (candidate_distance < nearest_distance) {
                    nearest_node = candidate_node;
                    nearest_distance = candidate_distance;
                }
            }

            // new node generation
            Node* new_node = rrt.steer(nearest_node, rand_node, nearest_distance);

            // if collision free then, add new_node in graph
            // connect edge b.w. new node and nearest node (store nearest node as parent node in new node)
            // if new node is close to target node, then generate path from init to target
            if (rrt.collision_free(nearest_node, new_node) == true) {
                new_node->parent = nearest_node;
                rrt.graph.add_node(new_node);
                addEdge(nearest_node, new_node, marker_pub, rrt.success);

                if (new_node->get_distance_to(rrt.target_x, rrt.target_y) < rrt.target_radius) {
                    Node* target_node = new Node(rrt.target_x, rrt.target_y);
                    rrt.graph.add_node(target_node);
                    target_node->parent = new_node;
                    rrt.success = true;
                    addEdge(new_node, target_node, marker_pub, rrt.success);
                }
            } else {
                delete new_node;
            }
        }

        else if (rrt.success && !done_path_drawing){
            drawFinalPath(rrt.graph, marker_pub);
            ROS_INFO_ONCE("target searching is successful");
            done_path_drawing = true;
        }

        else if (!rrt.success && sampling_cnt >= rrt.num_sampling) {
            ROS_INFO_ONCE("target searching is failed");
        }

        // iterate ROS
        ros::spinOnce();
        loop_rate.sleep();
        ++sampling_cnt;
    }
}

void initializeRviz(ros::Publisher marker_pub, const RRT& rrt) {
    visualization_msgs::Marker v_start, v_end;
    v_start.type = v_end.type = visualization_msgs::Marker::POINTS;
    v_start.header.frame_id = v_end.header.frame_id = "map";
    v_start.header.stamp = v_end.header.stamp = ros::Time::now();
    v_start.ns = v_end.ns = "start/end vertices";
    v_start.id = 0;
    v_end.id = 1;
    v_start.action = v_end.action = visualization_msgs::Marker::ADD;

    v_start.color.a = 1.0f;
    v_start.color.g = 1.0f;
    v_start.scale.x = 5;

    v_end.color.a = 1.0f;
    v_end.color.r = 1.0f;
    v_end.scale.x = 5;

    geometry_msgs::Point ps, pe;
    ps.x = rrt.start_x;
    ps.y = rrt.start_y;

    pe.x = rrt.target_x;
    pe.y = rrt.target_y;

    v_start.points.push_back(ps);
    v_end.points.push_back(pe);

    //publish edge and vertices
    marker_pub.publish(v_start);
    marker_pub.publish(v_end);

    populateObstacles(marker_pub, rrt);
}

void populateObstacles(ros::Publisher marker_pub, const RRT& rrt){
    int num_obs = 1;
    for (const auto& obstacle_pose: rrt.grid_map->get_obstacle_mid_point_set()){
        visualization_msgs::Marker obstacle;
        obstacle.type = visualization_msgs::Marker::CUBE;
        obstacle.header.frame_id = "map";
        obstacle.header.stamp = ros::Time::now();
        obstacle.ns = "obstacles";
        obstacle.action = visualization_msgs::Marker::ADD;
        obstacle.id = num_obs++;

        obstacle.scale.x = obstacle.scale.y = 20;
        obstacle.scale.z = 1;

        obstacle.pose.position.x = (obstacle_pose.first + 20);
        obstacle.pose.position.y = (obstacle_pose.second + 20);
        obstacle.pose.orientation.w = 1;

        obstacle.color.a = 1;
        obstacle.color.r = 6.6f;
        obstacle.color.g = 6.6f;
        obstacle.color.b = 6.6f;

        marker_pub.publish(obstacle);
    }
}

void addEdge(Node* from, Node* to, ros::Publisher marker_pub, bool success){
    static int num = 0;
    visualization_msgs::Marker edge;

    geometry_msgs::Point ps, pe;
    ps.x = from->x;
    ps.y = from->y;
    pe.x = to->x;
    pe.y = to->y;

    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "edges";
    edge.id = num++;

    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;
    edge.color.r = 1.0;

    if (!success) {
        edge.scale.x = 0.5;
        edge.color.r = 1.0;
    } else {
        edge.scale.x = 1;
        edge.color.g = edge.color.r = 1;
    }
    edge.color.a = 1.0;

    edge.points.push_back(ps);
    edge.points.push_back(pe);

    marker_pub.publish(edge);
}

void drawFinalPath(Graph graph, ros::Publisher marker_pub){
    // target node
    Node* child = *(--graph.get_node_list().end());
    Node* parent = child->parent;
    while (parent != nullptr){
        addEdge(child, parent, marker_pub, true);
        child = parent;
        parent = child->parent;
    }
}