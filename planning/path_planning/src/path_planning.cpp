#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <robo7_msgs/path.h>
#include <robo7_msgs/paths.h>
#include <robo7_msgs/trajectory.h>
#include <robo7_msgs/trajectory_point.h>
#include "robo7_srvs/IsGridOccupied.h"

class Node;
class PathPlanning;

typedef std::shared_ptr<Node> node_ptr;
float pi = 3.14159265358979323846;

int target_index = 2;
std::vector<float> x_targets = {.2, 1.6, 2.2, 2.2, .8};
std::vector<float> y_targets = {2.2, .8, 2.2, .2, .2};
float theta_target = pi / 2;

float x_target = x_targets[target_index];
float y_target = y_targets[target_index];

class Node
{
  public:
	std::shared_ptr<Node> parent;
	ros::ServiceClient occupancy_client;
	robo7_srvs::IsGridOccupied occupancy_srv;
	float x, y, theta;
	float angular_velocity, time, dt;
	float path_cost, cost_to_come, path_length, angular_velocity_resolution;
	float tolerance_radius, tolerance_angle;
	unsigned int node_id;
	std::vector<float> path_x, path_y, path_theta;

	Node(float x, float y, float theta, float angular_velocity, std::vector<float> path_x, std::vector<float> path_y, std::vector<float> path_theta, float path_cost, float cost_to_come, ros::ServiceClient occupancy_client, unsigned int node_id)
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

		this->path_length = 0.4;
		this->dt = 0.05;

		this->tolerance_radius = 5e-2;
		this->tolerance_angle = pi / 8;

		this->occupancy_client = occupancy_client;

		this->node_id = node_id;
	}

	float getCost()
	{
		//ROS_INFO("crC: %f, ctG: %f", cost_to_come, 20*getHeuristicCost());
		return cost_to_come + 5 * getHeuristicCost();
	}

	float getHeuristicCost()
	{
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
			return this->occupancy_srv.response.occupancy > 0.9;
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
	ros::Publisher paths_pub, start_goal_pub, goal_path_pub;
	ros::Subscriber robot_position;
	ros::ServiceClient occupancy_client;
	robo7_srvs::IsGridOccupied occupancy_srv;
	robo7_msgs::paths target_paths_msg;
	robo7_msgs::trajectory trajectory_msg;

	//Initialisation
	float goal_radius_tolerance, angle_tolerance;
	float x0, y0, theta0, position_updated;
	unsigned int node_id;

	PathPlanning(ros::NodeHandle nh, ros::Publisher paths_pub, ros::Publisher start_goal_pub)
	{
		this->nh = nh;
		this->paths_pub = paths_pub;
		this->start_goal_pub = start_goal_pub;
		robot_position = nh.subscribe("/localization/kalman_filter/position", 1000, &PathPlanning::getPositionCallBack, this);
		this->occupancy_client = nh.serviceClient<robo7_srvs::IsGridOccupied>("/occupancy_grid/is_occupied");

		goal_radius_tolerance = .03;
		angle_tolerance = 2 * pi;
		node_id = 1;
	}

	void getPositionCallBack(const geometry_msgs::Twist::ConstPtr &msg)
	{

		x0 = msg->linear.x;
		y0 = msg->linear.y;
		theta0 = msg->angular.z;

		position_updated = true;
	}

	std::vector<node_ptr> getSuccessorNodes(node_ptr node)
	{
		float steering_angle_max, angle_diff_tol, cost_to_come, cost;
		std::vector<node_ptr> successors;
		std::vector<float> path_x, path_y, path_theta;
		node_ptr node_target = std::make_shared<Node>(x_target, y_target, 0.0f, 0.0f, path_x, path_y, path_theta, 0.0f, 0.0f, occupancy_client, this->node_id++);

		steering_angle_max = pi;
		node->angular_velocity_resolution = pi / 2;

		angle_diff_tol = 1e-1;

		for (float angular_velocity = -steering_angle_max; angular_velocity <= steering_angle_max; angular_velocity += node->angular_velocity_resolution)
		{
			float x, y, theta, path_cost, t, dt, penalty_factor;
			bool add_node;
			x = node->x;
			y = node->y;
			theta = node->theta;

			if (angular_velocity == 0)
			{
				penalty_factor = 0.5;
			}
			else
				penalty_factor = 1.0;

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

				node_ptr successor_node = std::make_shared<Node>(x, y, theta, angular_velocity, path_x, path_y, path_theta, path_cost, cost_to_come, occupancy_client, this->node_id++);

				if (successor_node->inCollision())
				{
					add_node = false;
					break;
				}

				if (successor_node->distanceSquared(node_target) < this->goal_radius_tolerance && std::abs(successor_node->theta - theta_target) < this->angle_tolerance)
					break;

				this->occupancy_srv.request.x = x;
				this->occupancy_srv.request.y = y;

				if (this->occupancy_client.call(this->occupancy_srv))
				{
					cost = this->occupancy_srv.response.occupancy * penalty_factor;
					path_cost += cost;
					/*ROS_INFO("%f", cost);

					if (cost <= 0.4)
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
				cost_to_come += path_cost;
				node_ptr successor_node = std::make_shared<Node>(x, y, theta, angular_velocity, path_x, path_y, path_theta, path_cost, cost_to_come, occupancy_client, this->node_id++);
				successors.push_back(successor_node);
			}
		}

		return successors;
	}

	bool getPath()
	{
		std::vector<float> path_x, path_y, path_theta, goal_path_x, goal_path_y;
		std::vector<float> start_goal_x(2), start_goal_y(2), start_goal_theta(2);
		std::vector<node_ptr> successors, alive_nodes, dead_nodes;
		robo7_msgs::path path_msg, start_goal_msg;
		robo7_msgs::paths paths_msg;

		if (position_updated)
		{

			node_ptr node_start = std::make_shared<Node>(x0, y0, pi / 2, 0.0f, path_x, path_y, path_theta, 0.0f, 0.0f, occupancy_client, this->node_id++);
			node_ptr node_target = std::make_shared<Node>(x_target, y_target, 0.0f, 0.0f, path_x, path_y, path_theta, 0.0f, 0.0f, occupancy_client, this->node_id++);

			alive_nodes.push_back(node_start);

			start_goal_x[0] = x0;
			start_goal_x[1] = x_target;
			start_goal_y[0] = y0;
			start_goal_y[1] = y_target;

			start_goal_msg.path_x = start_goal_x;
			start_goal_msg.path_y = start_goal_y;

			while (!alive_nodes.empty())
			{
				start_goal_pub.publish(start_goal_msg);

				auto min_cost_iterator = std::min_element(alive_nodes.begin(), alive_nodes.end(), [](const node_ptr a, const node_ptr b) {
					return a->getCost() < b->getCost();
				});

				node_ptr node_current = alive_nodes[std::distance(alive_nodes.begin(), min_cost_iterator)];

				if (node_current->distanceSquared(node_target) < this->goal_radius_tolerance && std::abs(node_current->theta - theta_target) < this->angle_tolerance)
					return get_found_path(node_current);

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
		}
	}

	bool get_found_path(node_ptr node_current)
	{
		std::vector<node_ptr> goal_path_nodes, target_nodes;
		robo7_msgs::path goal_path_msg, target_path_msg;
		robo7_msgs::paths goal_paths_msg, target_paths_msg;
		robo7_msgs::trajectory_point trajectory_point_msg;
		robo7_msgs::trajectory trajectory_msg;
		bool search_done = false;

		ROS_INFO("Goal reached");
		search_done = true;

		target_nodes.push_back(node_current);

		while (node_current->parent->parent != NULL)
		{

			int partitions = (int)std::abs(node_current->angular_velocity / node_current->angular_velocity_resolution);

			node_ptr node_parent = node_current->parent;
			node_ptr partial_node = node_current;
			node_ptr partial_node_parent = node_parent;

			if (partitions > 0)
			{

				for (int i = partitions; i >= 0; i--)
				{

					std::vector<float>::const_iterator i_x_0 = node_current->path_x.begin() + i * (int)(node_current->path_x.size() / (partitions + 1));
					std::vector<float>::const_iterator i_x_end = i_x_0 + (int)(node_current->path_x.size() / (partitions + 1));

					std::vector<float>::const_iterator i_y_0 = node_current->path_y.begin() + i * (int)(node_current->path_y.size() / (partitions + 1));
					std::vector<float>::const_iterator i_y_end = i_y_0 + (int)(node_current->path_y.size() / (partitions + 1));

					std::vector<float>::const_iterator i_theta_0 = node_current->path_theta.begin() + i * (int)(node_current->path_theta.size() / (partitions + 1));
					std::vector<float>::const_iterator i_theta_end = i_theta_0 + (int)(node_current->path_theta.size() / (partitions + 1));

					std::vector<float> path_x(i_x_0, i_x_end);
					std::vector<float> path_y(i_y_0, i_y_end);
					std::vector<float> path_theta(i_theta_0, i_theta_end);

					float x = path_x[0];
					float y = path_y[0];
					float theta = path_theta[0];
					float angular_velocity = partial_node->angular_velocity;
					float path_cost = partial_node->path_cost / (partitions + 1);
					float cost_to_come = partial_node->cost_to_come;

					cost_to_come += path_cost;
					partial_node_parent = std::make_shared<Node>(x, y, theta, angular_velocity, path_x, path_y, path_theta, path_cost, cost_to_come, occupancy_client, this->node_id++);

					partial_node->parent = partial_node_parent;
					partial_node = partial_node_parent;
					target_nodes.push_back(partial_node);
				}
			}

			node_current = node_parent;
			target_nodes.push_back(node_current);
		}

		std::reverse(target_nodes.begin(), target_nodes.end());

		for (int i = 0; i < target_nodes.size(); i++)
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

		this->target_paths_msg = target_paths_msg;
		this->trajectory_msg = trajectory_msg;

		return search_done;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planning");

	ros::NodeHandle nh;
	nh = ros::NodeHandle("~");

	bool search_done = false;
	double control_frequency = 10.0;
	ros::Publisher paths_pub = nh.advertise<robo7_msgs::paths>("paths_vector", 1000);
	ros::Publisher start_goal_pub = nh.advertise<robo7_msgs::path>("start_goal", 1000);
	ros::Publisher target_path_pub = nh.advertise<robo7_msgs::paths>("target_path", 1000);
	ros::Publisher trajectory_pub = nh.advertise<robo7_msgs::trajectory>("trajectory", 1000);

	robo7_msgs::paths target_paths_msg;
	robo7_msgs::trajectory_point trajectory_point_msg;
	robo7_msgs::trajectory trajectory_msg;

	PathPlanning path_planning = PathPlanning(nh, paths_pub, start_goal_pub);

	ros::Rate loop_rate(control_frequency);

	while (nh.ok())
	{
		if (!search_done)
		{
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

	return 0;
}
