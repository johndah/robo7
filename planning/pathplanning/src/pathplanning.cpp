#include <math.h>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <robo7_msgs/path.h>
#include <robo7_msgs/paths.h>
#include "robo7_srvs/IsGridOccupied.h"

class PathPlanning;

bool path_found = false;
float pi = 3.14159265358979323846;
float x_target = 2.0, y_target = .2, theta_target = pi / 2;

class Node
{
  public:
	ros::ServiceClient occupancy_client;
	robo7_srvs::IsGridOccupied occupancy_srv;
	float x, y, theta;
	float control, time;
	float path_cost, cost_to_come;
	float tolerance_radius, tolerance_angle;
	Node *parent = NULL;
	std::vector<float> path_x, path_y;
	int alive_node_index;

	Node(float x, float y, float theta, float control, float time, std::vector<float> path_x, std::vector<float> path_y, float path_cost, float cost_to_come, ros::ServiceClient occupancy_client)
	{
		this->x = x;
		this->y = y;
		this->theta = theta;
		this->control = control;
		this->time = time;
		this->path_x = path_x;
		this->path_y = path_y;
		this->path_cost = path_cost;
		this->cost_to_come = cost_to_come;

		this->tolerance_radius = 1e-1;
		this->tolerance_angle = pi / 8;

		this->occupancy_client = occupancy_client;
	}

	float getCost()
	{
		//ROS_INFO("crC: %f, ctG: %f", cost_to_come, 20*getHeuristicCost());
		return cost_to_come + 20*getHeuristicCost();
	}

	float getHeuristicCost()
	{
		return sqrt(pow(x - x_target, 2.0) + pow(y - y_target, 2.0));
	}

	float distanceSquared(Node other_node)
	{
		return pow(this->x - other_node.x, 2.0) + pow(this->y - other_node.y, 2.0);
	}

	bool inCollision()
	{
		this->occupancy_srv.request.x = this->x;
		this->occupancy_srv.request.y = this->y;
		if (this->occupancy_client.call(this->occupancy_srv))
		{
			//ROS_INFO("x: %f   y: %f   occ: %f   coll: %d", this->occupancy_srv.request.x, this->occupancy_srv.request.y, this->occupancy_srv.response.occupancy, this->occupancy_srv.response.occupancy>0.9);
			return this->occupancy_srv.response.occupancy > 0.9;
		}
		else
		{
			return true;
		}
	}

	bool isClose(Node other)
	{
		float position = this->distanceSquared(other);
		float orientation = abs(theta - other.theta);

		return position <= tolerance_radius && orientation <= tolerance_angle;
	}

	bool is(Node other)
	{
		return x == other.x && y == other.y && theta == other.theta;
	}
};

double control_frequency = 10.0;

class PathPlanning
{
  public:
	ros::NodeHandle nh;
	ros::Publisher paths_pub, start_goal_pub, goal_path_pub;
	ros::Subscriber robot_position;
	ros::ServiceClient occupancy_client;
	robo7_srvs::IsGridOccupied occupancy_srv;

	float goal_radius_tolerance, angle_tolerance;

	//Initialisation
	float x0, y0, theta0, position_updated;

	PathPlanning(ros::NodeHandle nh, ros::Publisher paths_pub, ros::Publisher start_goal_pub, ros::Publisher goal_path_pub)
	{
		this->nh = nh;
		this->paths_pub = paths_pub;
		this->start_goal_pub = start_goal_pub;
		this->goal_path_pub = goal_path_pub;
		robot_position = nh.subscribe("/dead_reckoning/Pos", 1000, &PathPlanning::getPositionCallBack, this);
		this->occupancy_client = nh.serviceClient<robo7_srvs::IsGridOccupied>("/occupancy_grid/is_occupied");
		
		goal_radius_tolerance = .03;
		angle_tolerance = 2*pi;
	}

	void getPositionCallBack(const geometry_msgs::Twist::ConstPtr &msg)
	{

		x0 = msg->linear.x;
		y0 = msg->linear.y;
		theta0 = msg->angular.z;

		position_updated = true;
	}

	std::vector<Node> getSuccessorNodes(Node node)
	{
		float delta, delta_resolution, steering_angle_max, angle_diff_tol, angular_velocity_resolution, cost_to_come, path_length;
		std::vector<Node> successors;
		std::vector<float> path_x, path_y;
		Node node_target = Node(x_target, y_target, 0.0f, 0.0f, 0.0f, path_x, path_y, 0.0f, 0.0f, occupancy_client);


		path_length = 0.4;
		steering_angle_max = pi;
		angular_velocity_resolution = pi / 2;

		angle_diff_tol = 1e-1;

		for (float angular_velocity = -steering_angle_max; angular_velocity <= steering_angle_max; angular_velocity += angular_velocity_resolution)
		{
			float x, y, theta, path_cost, dt, t, penalty_factor;
			bool add_node;
			x = node.x;
			y = node.y;
			theta = node.theta;

			if (angular_velocity < angular_velocity_resolution){
				penalty_factor = 0.7;
			}
			else
				penalty_factor = 1.0;
			

			//angular_velocity = -pi / 4.0 * (float)((theta - delta > 0) - (theta - delta < -1.0e-3));
			//ROS_INFO("x: %f    y %f   th %f   delta %f   angular_velocity %f    path_cost: %f   t: %f", (float)((theta - delta > 0) - (theta - delta < 0)), y, theta, delta, angular_velocity, (float)(theta - delta > 0), theta - delta);

			t = 0.0;
			dt = 0.04;

			std::vector<float> path_x, path_y;
			path_cost = 0.0f;
			cost_to_come = node.cost_to_come;

			add_node = true;

			//while (std::abs(delta - theta) > angle_diff_tol)
			//for (int i = 0; i < 50; i++)
			while (t < path_length)
			{
				//ROS_INFO("x: %f    y %f   th %f   delta %f   angular_velocity %f    path_cost: %f   t: %f", x, y, theta, delta, angular_velocity, path_cost, t);
				float diff = std::abs(delta - theta);
				x += cos(theta) * dt;
				y += sin(theta) * dt;
				theta += angular_velocity * dt;

				t += dt;
				path_x.push_back(x);
				path_y.push_back(y);

				Node successor_node = Node(x, y, theta, angular_velocity, t, path_x, path_y, path_cost, cost_to_come, occupancy_client);

				if (successor_node.inCollision())
				{
					add_node = false;
					break;
				}
				
				if (successor_node.distanceSquared(node_target) < this->goal_radius_tolerance && std::abs(successor_node.theta - theta_target) < this->angle_tolerance)
					break;
				
				this->occupancy_srv.request.x = x;
				this->occupancy_srv.request.y = y;

				if (this->occupancy_client.call(this->occupancy_srv))
				{
					path_cost += this->occupancy_srv.response.occupancy*penalty_factor;
				}
				else
				{
					add_node = false;
					break;
				}
			}
			//ROS_INFO("delta: %f    theta %f ", delta, theta);

			if (add_node)
			{
				cost_to_come += path_cost;

				Node successor_node = Node(x, y, theta, angular_velocity, t, path_x, path_y, path_cost, cost_to_come, occupancy_client);
				successors.push_back(successor_node);
			}
		}

		return successors;
	}

	robo7_msgs::paths getPath()
	{
		std::vector<float> path_x, path_y, goal_path_x, goal_path_y;
		std::vector<float> start_goal_x(2), start_goal_y(2), start_goal_theta(2);
		std::vector<Node> successors, alive_nodes, dead_nodes;
		robo7_msgs::path path_msg, start_goal_msg, goal_path_msg;
		robo7_msgs::paths paths_msg, goal_paths_msg;

		if (position_updated)
		{
			Node node_start = Node(x0, y0, pi / 2, 0.0f, 0.0f, path_x, path_y, 0.0f, 0.0f, occupancy_client);
			Node node_target = Node(x_target, y_target, 0.0f, 0.0f, 0.0f, path_x, path_y, 0.0f, 0.0f, occupancy_client);

			alive_nodes.push_back(node_start);

			start_goal_x[0] = x0;
			start_goal_x[1] = x_target;
			start_goal_y[0] = y0;
			start_goal_y[1] = y_target;
			//start_goal_theta[0] = theta0;
			//start_goal_theta[1] = theta_target;

			start_goal_msg.path_x = start_goal_x;
			start_goal_msg.path_y = start_goal_y;
			//start_goal_msg.path_theta = start_goal_theta;

			while (!alive_nodes.empty())
			{
				start_goal_pub.publish(start_goal_msg);
				Node &node_current = alive_nodes[0];
				node_current.alive_node_index = 0;
				float cost = node_current.getCost();

				for (int i = 0; i < alive_nodes.size(); i++)
				{
					Node &node = alive_nodes[i];

					if (node.getCost() < cost)
					{
						node_current = node;
						node_current.alive_node_index = i;
						cost = node.getCost();
					}
				}

				if (node_current.distanceSquared(node_target) < this->goal_radius_tolerance && std::abs(node_current.theta - theta_target) < this->angle_tolerance)
				{
					ROS_INFO("Goal reached");
					path_found = true;

					std::vector<Node*> nodes;

					/*
					Node* node_c = &node_current;
					nodes.push_back(node_c);

					while (node_c->parent != NULL)
					{
						
						nodes.push_back(node_c);
						//Node *node_current = node_current->parent;
						node_c = node_c->parent;
						//ROS_INFO("%f", node_c->x);
						//break;
					}

					std::reverse(nodes.begin(), nodes.end());

					for (int i = 0; i < nodes.size(); i++)
					{
						Node *node = nodes[i];

						goal_path_msg.path_x = node->path_x;
						goal_path_msg.path_y = node->path_y;
						goal_paths_msg.paths.push_back(goal_path_msg);
					}

					goal_path_pub.publish(goal_paths_msg);
					*/

					break;
				}

				alive_nodes.erase(alive_nodes.begin() + node_current.alive_node_index);
				dead_nodes.push_back(node_current);

				successors = getSuccessorNodes(node_current);

				for (int i = 0; i < successors.size(); i++)
				{
					Node &node_successor = successors[i];
					node_successor.parent = &node_current;
					//node_successor.successor = node_current;

					bool in_dead_nodes = false;

					for (int k = 0; k < dead_nodes.size(); k++)
					{
						Node &dead_node = dead_nodes[k];

						//if (node_successor.inCollision())
						//ROS_INFO("Comparing x: %f=?%f  y: %f=?%f  th: %f=?%f", dead_node.x, node_successor.x, dead_node.y, node_successor.y, dead_node.theta, node_successor.theta);
						if (node_successor.is(dead_node))
						{
							in_dead_nodes = true;
							break;
						}
					}

					if (!in_dead_nodes)
					{

						if (node_successor.inCollision())
						{
							dead_nodes.push_back(node_successor);
						}
						else
						{
							bool match = false;

							for (int j = 0; j < alive_nodes.size(); j++)
							{
								Node &alive_node = alive_nodes[j];

								if (alive_node.isClose(node_successor) && node_successor.getCost() < alive_node.getCost())
								{
									match = true;
									alive_nodes.erase(alive_nodes.begin() + j);
									dead_nodes.push_back(node_successor);
									break;
								}
							}

							if (!match)
							{
								alive_nodes.push_back(node_successor);
								path_msg.path_x = node_successor.path_x;
								path_msg.path_y = node_successor.path_y;
								//std::vector<float> path_theta;
								//path_msg.path_theta = path_theta;
								paths_msg.paths.push_back(path_msg);
								paths_pub.publish(paths_msg);
							}
						}
					}
					else
					{
						ROS_INFO("Occupied x: %f, y: %f ", node_successor.x, node_successor.y);
					}
				}
			}

			return paths_msg;
		}

		std::vector<float> empty_path_x, empty_path_y;//, empty_path_theta;

		path_msg.path_x = empty_path_x;
		path_msg.path_y = empty_path_y;
		//path_msg.path_theta = empty_path_theta;
		paths_msg.paths.push_back(path_msg);

		return paths_msg;
	}

  private:
	float dt;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pathplanning");

	ros::NodeHandle nh;
	nh = ros::NodeHandle("~");

	ros::Publisher paths_pub = nh.advertise<robo7_msgs::paths>("paths_vector", 1000);
	ros::Publisher start_goal_pub = nh.advertise<robo7_msgs::path>("start_goal", 1000);
	ros::Publisher goal_path_pub = nh.advertise<robo7_msgs::paths>("goal_path", 1000);

	PathPlanning path_planning = PathPlanning(nh, paths_pub, start_goal_pub, goal_path_pub);

	ros::Rate loop_rate(control_frequency);

	robo7_msgs::paths paths_msg;

	while (nh.ok() && !path_found)
	{
		paths_msg = path_planning.getPath();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
