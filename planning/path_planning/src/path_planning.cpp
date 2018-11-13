#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <robo7_msgs/path.h>
#include <robo7_msgs/paths.h>
#include <robo7_msgs/trajectory.h>
#include <robo7_msgs/trajectory_point.h>
#include "robo7_srvs/IsGridOccupied.h"
#include "robo7_srvs/distanceTo.h"
#include <robo7_srvs/path_planning.h>

class Node;
class PathPlanning;

typedef std::shared_ptr<Node> node_ptr;
float pi = 3.14159265358979323846;

float x_target;
float y_target;

class Node
{
  public:
	std::shared_ptr<Node> parent;
	ros::ServiceClient occupancy_client, distance_client;
	robo7_srvs::IsGridOccupied occupancy_srv;
	robo7_srvs::distanceTo distance_srv;
	float x, y, theta;
	float angular_velocity, time, dt;
	float path_cost, cost_to_come, cost_to_go, path_length;
	float steering_angle_max, angular_velocity_resolution;
	float tolerance_radius, tolerance_angle;
	unsigned int node_id;
	std::vector<float> path_x, path_y, path_theta;

	Node(float x, float y, float theta, float angular_velocity, std::vector<float> path_x, std::vector<float> path_y, std::vector<float> path_theta, float path_cost, float cost_to_come, ros::ServiceClient occupancy_client, ros::ServiceClient distance_client, unsigned int node_id)
	{
		this->x = x;
		this->y = y;
		this->theta = theta;
		this->angular_velocity = angular_velocity;
		this->path_x = path_x;
		this->path_y = path_y;
		this->path_theta = path_theta;
		this->path_cost = path_cost;
		this->cost_to_come = cost_to_come;

		this->path_length = 0.3;
		this->steering_angle_max = 2 * pi;
		this->angular_velocity_resolution = pi;

		this->dt = 0.05;

		this->tolerance_radius = 3e-2;
		this->tolerance_angle = pi / 8;

		this->occupancy_client = occupancy_client;
		this->distance_client = distance_client;

		this->node_id = node_id;
	}

	float getCost()
	{
		ROS_INFO("crC: %f, ctG: %f", cost_to_come, cost_to_go/20);
		//ROS_INFO("crC: %f, ctG: %f", cost_to_come, 7*getHeuristicCost());
		//return cost_to_come + 7* sqrt(pow(x - x_target, 2.0) + pow(y - y_target, 2.0));//cost_to_go;

		return cost_to_come + cost_to_go / 20;
	}

	float getHeuristicCost()
	{
		this->distance_srv.request.x_from = x;
		this->distance_srv.request.y_from = y;
		this->distance_srv.request.x_to = x_target;
		this->distance_srv.request.y_to = y_target;

		if (this->distance_client.call(this->distance_srv))
		{
			//ROS_INFO("client %f    dist  %f", (float)this->distance_srv.response.distance, (float)sqrt(pow(x - x_target, 2.0) + pow(y - y_target, 2.0)));
			return this->distance_srv.response.distance;
		}
		else
			return sqrt(pow(x - x_target, 2.0) + pow(y - y_target, 2.0));
	}

	float distanceSquared(node_ptr other_node)
	{
		return pow(this->x - other_node->x, 2.0) + pow(this->y - other_node->y, 2.0);
	}

	bool inCollision()
	{
		this->occupancy_srv.request.x = this->x;
		this->occupancy_srv.request.y = this->y;
		if (this->occupancy_client.call(this->occupancy_srv))
		{
			return this->occupancy_srv.response.occupancy >= 1;
		}
		else
		{
			return true;
		}
	}

	bool isClose(node_ptr other)
	{
		float position = this->distanceSquared(other);
		float orientation = abs(theta - other->theta);

		return position <= tolerance_radius && orientation <= tolerance_angle;
	}

	bool is(node_ptr other)
	{
		return x == other->x && y == other->y && theta == other->theta;
	}
};

class PathPlanning
{
  public:
	ros::NodeHandle nh;
	ros::ServiceServer path_service;
	ros::Publisher paths_pub, target_pub, goal_path_pub, target_path_pub, trajectory_pub;
	ros::Subscriber robot_position;
	ros::ServiceClient occupancy_client, distance_client;
	robo7_srvs::IsGridOccupied occupancy_srv;
	robo7_srvs::distanceTo distance_srv;
	//robo7_msgs::trajectory_point trajectory_point_msg;
	//robo7_msgs::trajectory trajectory_msg;
	//robo7_msgs::paths target_paths_msg;

	//robo7_msgs::paths target_paths_msg;
	//robo7_msgs::trajectory_point trajectory_point_msg;
	//robo7_msgs::trajectory trajectory_msg;

	// Initialisation
	float goal_radius_tolerance;
	float x0, y0, theta0, x0_default, y0_default, theta0_default, position_updated;
	unsigned int node_id;

	PathPlanning(ros::NodeHandle nh, ros::Publisher paths_pub, ros::Publisher target_pub, ros::Publisher target_path_pub, ros::Publisher trajectory_pub)
	{
		this->nh = nh;
		this->paths_pub = paths_pub;
		this->target_pub = target_pub;
		this->target_path_pub = target_path_pub;
		this->trajectory_pub = trajectory_pub;

		// x0 = .215;
		// y0 = .2;

		robot_position = nh.subscribe("/localization/kalman_filter/position", 1000, &PathPlanning::getPositionCallBack, this);

		path_service = nh.advertiseService("path_service", &PathPlanning::getPath, this);

		this->occupancy_client = nh.serviceClient<robo7_srvs::IsGridOccupied>("/occupancy_grid/is_occupied");
		this->distance_client = nh.serviceClient<robo7_srvs::distanceTo>("/distance_grid/distance");

		goal_radius_tolerance = .05;
		node_id = 1;
	}

	void getPositionCallBack(const geometry_msgs::Twist::ConstPtr &msg)
	{

		x0_default = msg->linear.x;
		y0_default = msg->linear.y;
		theta0_default = msg->angular.z;

		position_updated = true;
	}

	std::vector<node_ptr> getSuccessorNodes(node_ptr node)
	{
		// float steering_angle_max,
		float angle_diff_tol, cost_to_come, cost, path_length_max;
		std::vector<node_ptr> successors;
		std::vector<float> path_x, path_y, path_theta;
		node_ptr node_target = std::make_shared<Node>(x_target, y_target, 0.0f, 0.0f, path_x, path_y, path_theta, 0.0f, 0.0f, occupancy_client, distance_client, this->node_id++);

		angle_diff_tol = 1e-1;

		for (float angular_velocity = -node->steering_angle_max; angular_velocity <= node->steering_angle_max; angular_velocity += node->angular_velocity_resolution)
		{
			if (std::abs(angular_velocity) < 1e-1)
				path_length_max = 0.4;
			else
				path_length_max = 0.3;

			//for (float path_length = 0.3; path_length <= path_length_max; path_length += 0.1)
			//{
			float x, y, theta, path_cost, t, dt, penalty_factor;
			bool add_node;
			x = node->x;
			y = node->y;
			theta = node->theta;

			if (std::abs(angular_velocity) < 1e-1)
			{
				penalty_factor = 0.2;
				node->path_length = 0.4;
				node->steering_angle_max = pi;
				node->angular_velocity_resolution = pi/2;
			}
			else if (std::abs(angular_velocity - node->angular_velocity_resolution) < 1e-1)
			{
				penalty_factor = .8;
				node->path_length = 0.3;
				node->steering_angle_max = 2*pi;
				node->angular_velocity_resolution = pi/2;
			}
			else
			{
				penalty_factor = 1.0; 
				node->path_length = 0.2;
				node->steering_angle_max = 3 * pi;
				node->angular_velocity_resolution = pi/2;
			}

			t = 0.0;
			dt = node->dt;

			std::vector<float> path_x, path_y;
			path_cost = 0.0f;
			cost_to_come = node->cost_to_come;

			add_node = true;

			while (t < node->path_length)
			{
				// ROS_INFO("x: %f    y %f   th %f    angular_velocity %f    path_cost: %f   t: %f", x, y, theta, angular_velocity, path_cost, t);
				x += cos(theta) * dt;
				y += sin(theta) * dt;
				theta += angular_velocity * dt;

				t += dt;
				path_x.push_back(x);
				path_y.push_back(y);
				path_theta.push_back(theta);

				node_ptr successor_node = std::make_shared<Node>(x, y, theta, angular_velocity, path_x, path_y, path_theta, path_cost, cost_to_come, occupancy_client, distance_client, this->node_id++);
				successor_node->cost_to_go = successor_node->getHeuristicCost();

				if (successor_node->inCollision())
				{
					add_node = false;
					break;
				}

				if (successor_node->distanceSquared(node_target) < this->goal_radius_tolerance)
					break;

				this->occupancy_srv.request.x = x;
				this->occupancy_srv.request.y = y;

				if (this->occupancy_client.call(this->occupancy_srv))
				{
					cost = this->occupancy_srv.response.occupancy * node-> path_length * penalty_factor;

					path_cost += cost;
					/*
					if (cost <= .6)
						node->path_length = 0.4;
					else
						node->path_length = 0.3;
					*/
				}
				else
				{
					add_node = false;
					break;
				}
			}

			if (add_node)
			{
				/*
				if (this->occupancy_client.call(this->occupancy_srv))
				{
					cost = this->occupancy_srv.response.occupancy * penalty_factor;
					path_cost += cost;
					if (cost <= 0.6)
					{

						node->path_length = 0.4;
						node->steering_angle_max = pi;
						node->angular_velocity_resolution = pi/2;
					}
					else
					{	node->path_length = 0.3;
						node->steering_angle_max = 2*pi;
						node->angular_velocity_resolution = pi;
					}
				}
					*/
				cost_to_come += path_cost;

				node_ptr successor_node = std::make_shared<Node>(x, y, theta, angular_velocity, path_x, path_y, path_theta, path_cost, cost_to_come, occupancy_client, distance_client, this->node_id++);
				successor_node->cost_to_go = successor_node->getHeuristicCost();

				successors.push_back(successor_node);
			}
			//}
		}

		return successors;
	}

	bool getPath(robo7_srvs::path_planning::Request &req, robo7_srvs::path_planning::Response &res)
	{
		std::vector<float> path_x, path_y, path_theta, goal_path_x, goal_path_y;
		std::vector<node_ptr> successors, alive_nodes, dead_nodes;
		robo7_msgs::path path_msg, target_path_msg;
		geometry_msgs::Point target_msg;
		robo7_msgs::paths target_paths_msg;
		robo7_msgs::paths paths_msg;
		geometry_msgs::Twist robot_position = req.robot_position;
		geometry_msgs::Point destination_position = req.destination_position;

		/*
		target_path_msg.path_x = path_x;
		target_path_msg.path_y = path_y;
		target_paths_msg.paths.push_back(target_path_msg);
		ROS_INFO("Removing paths");
		for (int i = 0; i < 10; i++)
		{
			target_path_pub.publish(target_paths_msg);
		}
		*/

		x0 = robot_position.linear.x;
		y0 = robot_position.linear.y;
		theta0 = robot_position.angular.z;

		if (position_updated && x0 == 0 && y0 == 0 && theta0 == 0)
		{
			x0 = x0_default;
			y0 = y0_default;
			theta0 = theta0_default;
		}

		x_target = destination_position.x;
		y_target = destination_position.y;

		//robo7_msgs::trajectory_point aPoint;
		//robo7_msgs::trajectory all_points;

		//if (position_updated){

		node_ptr node_target = std::make_shared<Node>(x_target, y_target, 0.0f, 0.0f, path_x, path_y, path_theta, 0.0f, 0.0f, occupancy_client, distance_client, this->node_id++);
		node_target->cost_to_go = 0;

		float theta0_resolution = pi / 2;

		for (float t0 = theta0 - pi; t0 < theta0 + pi; t0 += theta0_resolution)
		//for (float theta0 = -pi; theta0 < pi; theta0 += theta0_resolution)
		{
			//ROS_INFO("Theta0 %f", theta0);
			node_ptr node_start = std::make_shared<Node>(x0, y0, t0, 0.0f, path_x, path_y, path_theta, 0.0f, 0.0f, occupancy_client, distance_client, this->node_id++);
			node_start->cost_to_go = node_start->getHeuristicCost();
			alive_nodes.push_back(node_start);
		}

		target_msg.x = x_target;
		target_msg.y = y_target;

		while (!alive_nodes.empty())
		{
			target_pub.publish(target_msg);

			auto min_cost_iterator = std::min_element(alive_nodes.begin(), alive_nodes.end(), [](const node_ptr a, const node_ptr b) {
				return a->getCost() < b->getCost();
			});

			node_ptr node_current = alive_nodes[std::distance(alive_nodes.begin(), min_cost_iterator)];

			//ROS_INFO("Distance %f  Required distance: %f", node_current->distanceSquared(node_target),  this->goal_radius_tolerance);
			if (node_current->distanceSquared(node_target) < this->goal_radius_tolerance) // && std::abs(node_current->theta - theta_target) < this->angle_tolerance)
				return get_found_path(node_current, res);

			alive_nodes.erase(min_cost_iterator);
			dead_nodes.push_back(node_current);

			successors = getSuccessorNodes(node_current);

			for (int i = 0; i < successors.size(); i++)
			{
				node_ptr node_successor = successors[i];
				node_successor->parent = node_current;

				bool in_dead_nodes = false;

				/*if (std::none_of(dead_nodes.begin(), dead_nodes.end(), is(dead_node) ))
					{
						ROS_INFO("2 id : %d", node_successor->node_id);
					}*/

				for (int k = 0; k < dead_nodes.size(); k++)
				{
					node_ptr dead_node = dead_nodes[k];

					if (node_successor->is(dead_node))
					{
						in_dead_nodes = true;
						break;
					}
				}

				if (!in_dead_nodes)
				{
					bool match = false;

					for (int j = 0; j < alive_nodes.size(); j++)
					{
						node_ptr alive_node = alive_nodes[j];

						if (alive_node->is(node_successor) && node_successor->getCost() < alive_node->getCost())
						{
							match = true;
							alive_nodes.erase(alive_nodes.begin() + j);
							alive_nodes.push_back(node_successor);
							break;
						}
					}

					if (!match)
					{
						alive_nodes.push_back(node_successor);
						path_msg.path_x = node_successor->path_x;
						path_msg.path_y = node_successor->path_y;
						paths_msg.paths.push_back(path_msg);
						paths_pub.publish(paths_msg);
					}
				}
			}
		}
		ROS_INFO("Path not found");
		// return search_done;
		//}
	}

	bool get_found_path(node_ptr node_current, robo7_srvs::path_planning::Response &res)
	{
		std::vector<node_ptr> target_nodes;
		robo7_msgs::path target_path_msg;
		robo7_msgs::paths target_paths_msg;
		robo7_msgs::trajectory_point trajectory_point_msg;
		robo7_msgs::trajectory trajectory_msg;
		bool search_done = false;

		ROS_INFO("Goal reached");
		search_done = true;

		target_nodes.push_back(node_current);

		while (node_current->parent != NULL)
		{

			int partitions = 2;

			// int partitions = (int) std::abs(node_current->angular_velocity / node_current->angular_velocity_resolution);
			//node_current->path_length / 0.1 +  (int)std::abs(node_current->angular_velocity / node_current->angular_velocity_resolution);
			//if (partitions == 0)
			//	partitions += 2;

			node_ptr node_parent = node_current->parent;
			node_ptr partial_node = node_current;
			node_ptr partial_node_parent = node_parent;

			if (partitions >= 0 && node_current->path_x.size() >= 3)
			{

				for (int i = partitions; i >= 0; i--)
				{
					int parts = partitions + 1;
					std::vector<float>::const_iterator i_x_0 = node_current->path_x.begin() + i * (int)(node_current->path_x.size() / parts);
					std::vector<float>::const_iterator i_x_end = i_x_0 + (int)(node_current->path_x.size() / parts);

					std::vector<float>::const_iterator i_y_0 = node_current->path_y.begin() + i * (int)(node_current->path_y.size() / parts);
					std::vector<float>::const_iterator i_y_end = i_y_0 + (int)(node_current->path_y.size() / parts);

					std::vector<float>::const_iterator i_theta_0 = node_current->path_theta.begin() + i * (int)(node_current->path_theta.size() / parts);
					std::vector<float>::const_iterator i_theta_end = i_theta_0 + (int)(node_current->path_theta.size() / parts);

					std::vector<float> path_x(i_x_0, i_x_end);
					std::vector<float> path_y(i_y_0, i_y_end);
					std::vector<float> path_theta(i_theta_0, i_theta_end);

					float x = path_x[0];
					float y = path_y[0];
					float theta = path_theta[0];
					float angular_velocity = partial_node->angular_velocity;
					float path_cost = partial_node->path_cost / parts;
					float cost_to_come = partial_node->cost_to_come;
					//float cost_to_go = partial_node->getHeuristicCost();

					cost_to_come += path_cost;
					partial_node_parent = std::make_shared<Node>(x, y, theta, angular_velocity, path_x, path_y, path_theta, path_cost, cost_to_come, occupancy_client, distance_client, this->node_id++);
					partial_node_parent->cost_to_go = partial_node_parent->getHeuristicCost();

					partial_node->parent = partial_node_parent;
					partial_node = partial_node_parent;
					target_nodes.push_back(partial_node);
				}
			}

			this->occupancy_srv.request.x = node_current->x;
			this->occupancy_srv.request.y = node_current->y;

			if (this->occupancy_client.call(this->occupancy_srv))
			{
				float cost = this->occupancy_srv.response.occupancy;

				ROS_INFO("%f", cost);
				/*
					if (cost/node->path_length <= 1)
						node->path_length = 0.4;
					else
						node->path_length = 0.3;
					*/
			}

			node_current = node_parent;
			target_nodes.push_back(node_current);
		}

		std::reverse(target_nodes.begin(), target_nodes.end());

		for (int i = 1; i < target_nodes.size(); i++)
		{
			node_ptr node = target_nodes[i];

			float partitions = std::abs(node->angular_velocity / node->angular_velocity_resolution);

			target_path_msg.path_x = node->path_x;
			target_path_msg.path_y = node->path_y;
			target_paths_msg.paths.push_back(target_path_msg);

			trajectory_point_msg.id_number = i;
			trajectory_point_msg.point_coord.x = node->x;
			trajectory_point_msg.point_coord.y = node->y;
			trajectory_point_msg.point_coord.z = 0;
			trajectory_point_msg.speed = .15 - .05 * partitions;
			trajectory_point_msg.distance = node->path_length / (partitions + 1);
			trajectory_msg.trajectory_points.push_back(trajectory_point_msg);
		}

		//this->target_paths_msg = target_paths_msg;
		//this->trajectory_msg = trajectory_msg;

		for (int i = 0; i < 10; i++)
		{
			target_path_pub.publish(target_paths_msg);
			trajectory_pub.publish(trajectory_msg);
		}

		res.path_planned = trajectory_msg;
		res.success = search_done;

		return search_done;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planning");

	ros::NodeHandle nh;
	nh = ros::NodeHandle("~");

	bool search_done = false;
	double control_frequency = 1.0;
	ros::Publisher paths_pub = nh.advertise<robo7_msgs::paths>("paths_vector", 1000);
	//ros::Publisher start_goal_pub = nh.advertise<robo7_msgs::path>("start_goal", 1000);
	ros::Publisher target_pub = nh.advertise<geometry_msgs::Point>("target", 1000);
	ros::Publisher target_path_pub = nh.advertise<robo7_msgs::paths>("target_path", 1000);
	ros::Publisher trajectory_pub = nh.advertise<robo7_msgs::trajectory>("trajectory", 1000);

	/*
	robo7_msgs::paths target_paths_msg;
	robo7_msgs::trajectory_point trajectory_point_msg;
	robo7_msgs::trajectory trajectory_msg;
	*/

	ROS_INFO("Init path_planning");

	PathPlanning path_planning = PathPlanning(nh, paths_pub, target_pub, target_path_pub, trajectory_pub);

	ros::Rate loop_rate(control_frequency);

	/*
	while (nh.ok())
	{
		if (!search_done)
		{
			// Will do this to a service
			search_done = path_planning.getPath();
		}
		else
		{
			target_path_pub.publish(path_planning.target_paths_msg);
			trajectory_pub.publish(path_planning.trajectory_msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
		*/

	ros::spin();

	return 0;
}
