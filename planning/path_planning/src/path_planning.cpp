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
#include <robo7_msgs/target_trajectory.h>
#include <robo7_msgs/target_trajectory_point.h>
#include "robo7_srvs/IsGridOccupied.h"
#include "robo7_srvs/distanceTo.h"
#include <robo7_srvs/path_planning.h>
#include <Eigen/Dense>

class Node;
class PathPlanning;

typedef std::shared_ptr<Node> node_ptr;
float pi = 3.14159265358979323846;

float x_target;
float y_target;
float exploration;

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

		this->dt = 0.05;

		this->path_length = 0.3;
		if (exploration)
			this->steering_angle_max = pi / (5.0 * this->dt);
		else
			this->steering_angle_max = pi / (10.0 * this->dt);

		this->angular_velocity_resolution = pi / 2;

		this->tolerance_radius = 3e-2;
		this->tolerance_angle = pi / 8.0;

		this->occupancy_client = occupancy_client;
		this->distance_client = distance_client;

		this->node_id = node_id;
	}

	float getCost()
	{
		return cost_to_come + cost_to_go / 20;
	}

	float getHeuristicCost()
	{
		this->distance_srv.request.x_from = x;
		this->distance_srv.request.y_from = y;
		this->distance_srv.request.x_to = x_target;
		this->distance_srv.request.y_to = y_target;

		if (this->distance_client.call(this->distance_srv))
			return this->distance_srv.response.distance;
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
			return this->occupancy_srv.response.occupancy >= 1;
		else
			return true;
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
	ros::Publisher paths_pub, target_pub, goal_path_pub, target_path_pub, trajectory_pub, target_trajectory_pub;
	ros::Subscriber robot_position;
	ros::ServiceClient occupancy_client, distance_client;
	robo7_srvs::IsGridOccupied occupancy_srv;
	robo7_srvs::distanceTo distance_srv;

	// Initialisation
	float goal_radius_tolerance;
	float x0, y0, theta0, x0_default, y0_default, theta0_default, position_updated;
	bool field_scale;
	unsigned int node_id;

	PathPlanning(ros::NodeHandle nh, ros::Publisher paths_pub, ros::Publisher target_pub, ros::Publisher target_path_pub, ros::Publisher trajectory_pub, ros::Publisher target_trajectory_pub)
	{
		this->nh = nh;
		this->paths_pub = paths_pub;
		this->target_pub = target_pub;
		this->target_path_pub = target_path_pub;
		this->trajectory_pub = trajectory_pub;
		this->target_trajectory_pub = target_trajectory_pub;

		robot_position = nh.subscribe("localization/kalman_filter/position_timed", 1000, &PathPlanning::getPositionCallBack, this);
		// robot_position = nh.subscribe("/localization/kalman_filter/position", 1000, &PathPlanning::getPositionCallBack, this);

		path_service = nh.advertiseService("path_service", &PathPlanning::getPath, this);

		this->occupancy_client = nh.serviceClient<robo7_srvs::IsGridOccupied>("/occupancy_grid/is_occupied");
		this->distance_client = nh.serviceClient<robo7_srvs::distanceTo>("/distance_grid/distance");

		goal_radius_tolerance = .02;
		node_id = 1;
	}

	void getPositionCallBack(const geometry_msgs::Twist::ConstPtr &msg)
	{

		x0_default = msg->linear.x;
		y0_default = msg->linear.y;
		theta0_default = msg->angular.z;

		position_updated = true;
	}

	std::vector<node_ptr> getSuccessorNodes(node_ptr node, node_ptr node_target)
	{
		float angle_diff_tol, cost_to_come, cost, path_length_max;
		std::vector<node_ptr> successors;
		std::vector<float> path_x, path_y, path_theta;
		// node_ptr node_target = std::make_shared<Node>(x_target, y_target, 0.0f, 0.0f, path_x, path_y, path_theta, 0.0f, 0.0f, occupancy_client, distance_client, this->node_id++);

		angle_diff_tol = 1e-1;

		for (float angular_velocity = -node->steering_angle_max; angular_velocity <= node->steering_angle_max; angular_velocity += node->angular_velocity_resolution)
		{
			if (std::abs(angular_velocity) < 1e-1)
				path_length_max = 0.4;
			else
				path_length_max = 0.3;

			float x, y, theta, path_cost, t, dt, penalty_factor;
			bool add_node;
			x = node->x;
			y = node->y;
			theta = node->theta;

			if (std::abs(angular_velocity) < 1e-1)
			{
				penalty_factor = 0.3;
				node->path_length = 0.4 * this->field_scale;
				// ROS_INFO("Length %f, ", node->path_length);
			}
			else if (std::abs(angular_velocity) - node->angular_velocity_resolution < 1e-1 || std::abs(angular_velocity) - 2 * node->angular_velocity_resolution < 1e-1)
			{
				penalty_factor = .7;
				node->path_length = 0.3 * this->field_scale;
			}
			else
			{
				penalty_factor = 1.0;
				node->path_length = 0.25 * this->field_scale;
			}

			t = 0.0;
			dt = node->dt;

			std::vector<float> path_x, path_y, path_theta;
			path_cost = 0.0;
			cost_to_come = node->cost_to_come;

			add_node = true;

			while (t < node->path_length)
			{
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

				occupancy_srv.request.x = x;
				occupancy_srv.request.y = y;

				if (occupancy_client.call(occupancy_srv))
					path_cost = occupancy_srv.response.occupancy * node->path_length * penalty_factor;
				else
				{
					add_node = false;
					break;
				}
			}

			if (add_node)
			{
				cost_to_come += path_cost;
				node_ptr successor_node = std::make_shared<Node>(x, y, theta, angular_velocity, path_x, path_y, path_theta, path_cost, cost_to_come, occupancy_client, distance_client, this->node_id++);
				successor_node->cost_to_go = successor_node->getHeuristicCost();

				successors.push_back(successor_node);
			}
		}

		return successors;
	}

	node_ptr getDirectTarget(node_ptr node, float x_diff, float y_diff)
	{
		std::vector<float> path_x, path_y, path_theta;
		float path_length, angular_velocity, cost_to_come, penalty_factor;

		path_length = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

		float x, y, theta, path_cost, t, dt;
		x = node->x;
		y = node->y;
		theta = std::fmod(atan2(y_diff, x_diff) + pi, 2 * pi) - pi;
		// ROS_INFO("xdiff, y_diff, theta %f, path_length %f", x_diff, y_diff, theta, path_length);
		angular_velocity = 0;
		penalty_factor = 0.4;
		t = 0.0;
		dt = node->dt;

		path_cost = 0.0;
		cost_to_come = node->cost_to_come;

		// add_node = true;

		while (t < path_length)
		{
			x += cos(theta) * dt;
			y += sin(theta) * dt;
			theta += angular_velocity * dt;

			t += dt;
			path_x.push_back(x);
			path_y.push_back(y);
			path_theta.push_back(theta);

			occupancy_srv.request.x = x;
			occupancy_srv.request.y = y;

			if (occupancy_client.call(occupancy_srv))
				path_cost = occupancy_srv.response.occupancy * path_length * penalty_factor;
		}

		// ROS_INFO("Adding direct target at x %f  y %f", x, y);
		cost_to_come += path_cost;
		node_ptr successor_node = std::make_shared<Node>(x, y, theta, angular_velocity, path_x, path_y, path_theta, path_cost, cost_to_come, occupancy_client, distance_client, this->node_id++);
		successor_node->cost_to_go = successor_node->getHeuristicCost();

		return successor_node;
	}

	bool targetInSight(node_ptr node_current, node_ptr node_target, robo7_srvs::path_planning::Response &res)
	{
		float x_diff, y_diff, x_ray, y_ray, path_length;

		x_diff = float(node_target->x - node_current->x);
		y_diff = float(node_target->y - node_current->y);

		int n = floor(200 * std::max(std::abs(x_diff), std::abs(y_diff)));
		ROS_INFO("n %d  xc %f yc %f   xt %f  yt %f  xdiff %f ydiff %f", n,  node_current->x,  node_current->y, node_target->x, node_target->y, x_diff, y_diff);
		bool visable = true;

		x_ray = node_current->x;
		y_ray = node_current->y;

		for (int i_ray = 0; i_ray < n; i_ray++)
		{
			x_ray += x_diff / n;
			y_ray += y_diff / n;

			occupancy_srv.request.x = x_ray;
			occupancy_srv.request.y = y_ray;
			if (occupancy_client.call(occupancy_srv))
			{
				//ROS_INFO("Ray:  x %f y %f  occ %f", x_ray, y_ray, occupancy_srv.response.occupancy);
				if (occupancy_srv.response.occupancy == 1.0)
				{
					visable = false;
					break;
				}
			}
		}

		if (visable)
		{
			node_ptr node_successor = getDirectTarget(node_current, x_diff, y_diff);
			node_successor->parent = node_current;

			get_found_path(node_successor, node_successor, res);
		}

		return visable;
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

		// ROS_INFO("EC %d", req.exploring);
		if (req.exploring)
			this->field_scale = 1.0;
		else
			this->field_scale = 1.0;

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

		path_x.push_back(x_target);
		path_y.push_back(y_target);

		node_ptr node_target = std::make_shared<Node>(x_target, y_target, 0.0f, 0.0f, path_x, path_y, path_theta, 0.0f, 0.0f, occupancy_client, distance_client, this->node_id++);
		node_target->cost_to_go = 0;

		path_x.clear();
		path_y.clear();
		path_theta.clear();

		float theta0_resolution;

		if (exploration)
			theta0_resolution = pi / 4;
		else
			theta0_resolution = pi / 2;

		for (float t0 = theta0 - pi; t0 < theta0 + pi; t0 += theta0_resolution)
		{
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

			if (targetInSight(node_current, node_target, res))
				return true;

			if (node_current->distanceSquared(node_target) < this->goal_radius_tolerance)
				return get_found_path(node_current, node_current, res);

			alive_nodes.erase(min_cost_iterator);
			dead_nodes.push_back(node_current);

			successors = getSuccessorNodes(node_current, node_target);

			for (int i = 0; i < successors.size(); i++)
			{
				node_ptr node_successor = successors[i];
				node_successor->parent = node_current;

				bool in_dead_nodes = false;

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

		res.success = false;
		return false;
	}

	bool get_found_path(node_ptr node_current, node_ptr node_target, robo7_srvs::path_planning::Response &res)
	{
		std::vector<node_ptr> target_nodes;
		robo7_msgs::path target_path_msg;
		robo7_msgs::paths target_paths_msg;
		robo7_msgs::trajectory_point trajectory_point_msg;
		robo7_msgs::target_trajectory_point target_trajectory_point_msg;
		robo7_msgs::trajectory trajectory_msg;
		robo7_msgs::target_trajectory target_trajectory_msg;
		bool search_done;

		ROS_INFO("Path found!");

		search_done = true;

		target_nodes.push_back(node_current);

		while (node_current->parent != NULL)
		{

			int partitions = 2;

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

					cost_to_come += path_cost;
					partial_node_parent = std::make_shared<Node>(x, y, theta, angular_velocity, path_x, path_y, path_theta, path_cost, cost_to_come, occupancy_client, distance_client, this->node_id++);
					partial_node_parent->cost_to_go = partial_node_parent->getHeuristicCost();

					partial_node->parent = partial_node_parent;
					partial_node = partial_node_parent;
					target_nodes.push_back(partial_node);
				}
			}

			// this->occupancy_srv.request.x = node_current->x;
			// this->occupancy_srv.request.y = node_current->y;

			node_current = node_parent;
			target_nodes.push_back(node_current);
		}

		std::reverse(target_nodes.begin(), target_nodes.end());

		node_target->path_theta.push_back(node_target->theta);

		target_nodes.push_back(node_target);

		for (int i = 1; i < target_nodes.size(); i++)
		{
			node_ptr node = target_nodes[i];

			float partitions = std::abs(node->angular_velocity / node->angular_velocity_resolution);

			target_path_msg.path_x = node->path_x;
			target_path_msg.path_y = node->path_y;
			target_paths_msg.paths.push_back(target_path_msg);

			if (!node->path_x.empty())
			{
				trajectory_point_msg.id_number = i;
				trajectory_point_msg.point_coord.x = node->path_x[node->path_x.size() - 1];
				trajectory_point_msg.point_coord.y = node->path_y[node->path_y.size() - 1];
				trajectory_point_msg.point_coord.z = node->path_theta[node->path_theta.size() - 1];

				trajectory_point_msg.pose.linear.x = node->path_x[node->path_x.size() - 1];
				trajectory_point_msg.pose.linear.y = node->path_y[node->path_y.size() - 1];
				trajectory_point_msg.pose.angular.z = node->path_theta[node->path_theta.size() - 1];

				trajectory_point_msg.speed = .15 - .05 * partitions;
				trajectory_point_msg.distance = node->path_length / (partitions + 1);
				trajectory_msg.trajectory_points.push_back(trajectory_point_msg);

				if (node->parent != NULL)
				{

					target_trajectory_point_msg.id_number = i;

					float x, y, theta, x_successor, y_successor, theta_successor, theta_diff;

					x = node->parent->path_x[node->parent->path_x.size() - 1];
					y = node->parent->path_y[node->parent->path_y.size() - 1];
					theta = node->parent->path_theta[node->parent->path_theta.size() - 1];

					x_successor = node->path_x[node->path_x.size() - 1];
					y_successor = node->path_y[node->path_y.size() - 1];
					theta_successor = node->path_theta[node->path_theta.size() - 1];

					target_trajectory_point_msg.starting_point.linear.x = x;
					target_trajectory_point_msg.starting_point.linear.y = y;
					target_trajectory_point_msg.starting_point.angular.z = theta;

					target_trajectory_point_msg.end_point.linear.x = x_successor;
					target_trajectory_point_msg.end_point.linear.y = y_successor;
					target_trajectory_point_msg.end_point.angular.z = theta_successor;

					theta_diff = std::abs(std::fmod(theta_successor - theta + pi, 2 * pi) - pi);

					if (theta_diff == 0.0)
					{
						target_trajectory_point_msg.is_it_line = true;
						target_trajectory_point_msg.curve_radius = 0;
					}
					else
					{
						target_trajectory_point_msg.is_it_line = false;
						target_trajectory_point_msg.curve_radius = sqrt(node->distanceSquared(node->parent)) / (2 * sin(theta_diff / 2));
					}
					target_trajectory_msg.target_trajectory_points.push_back(target_trajectory_point_msg);
				}
			}
		}

		target_path_pub.publish(target_paths_msg);
		trajectory_pub.publish(trajectory_msg);
		target_trajectory_pub.publish(target_trajectory_msg);

		geometry_msgs::Twist destination_pose;
		destination_pose.linear.x = target_nodes[target_nodes.size() - 1]->x;
		destination_pose.linear.y = target_nodes[target_nodes.size() - 1]->y;
		destination_pose.angular.z = target_nodes[target_nodes.size() - 1]->theta;

		res.path_planned = trajectory_msg;
		res.path = target_trajectory_msg;
		res.destination_pose = destination_pose;
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
	ros::Publisher target_pub = nh.advertise<geometry_msgs::Point>("target", 1000);
	ros::Publisher target_path_pub = nh.advertise<robo7_msgs::paths>("target_path", 1000);
	ros::Publisher trajectory_pub = nh.advertise<robo7_msgs::trajectory>("trajectory", 1000);
	ros::Publisher target_trajectory_pub = nh.advertise<robo7_msgs::target_trajectory>("target_trajectory", 1000);

	ROS_INFO("Init path_planning");

	PathPlanning path_planning = PathPlanning(nh, paths_pub, target_pub, target_path_pub, trajectory_pub, target_trajectory_pub);

	ros::Rate loop_rate(control_frequency);

	ros::spin();

	return 0;
}