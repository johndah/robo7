#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <robo7_msgs/path.h>
#include <robo7_msgs/paths.h>

class PathPlanning;

float xt = 3, yt = 3;
float pi = 3.14159265358979323846;

class Node
{
  public:
	//template <class T>;
	float x, y, theta;
	float control, time;
	float path_cost, cost_to_come;
	float tolerance_radius, tolerance_angle;
	Node *parent;
	std::vector<float> path_x, path_y;
	int alive_node_index;

	Node(float x, float y, float theta, float control, float time, std::vector<float> path_x, std::vector<float> path_y, float path_cost, float cost_to_come)
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

		this->tolerance_radius = 3e-3;
		this->tolerance_angle = pi / 8;
	}

	float getCost()
	{
		return cost_to_come + getHeuristicCost();
	}

	float getHeuristicCost()
	{
		return sqrt(pow(x - xt, 2.0) + pow(y - yt, 2.0));
	}

	float distanceSquared(Node other_node)
	{
		return pow(this->x - other_node.x, 2.0) + pow(this->y - other_node.y, 2.0);
	}

	bool inCollision()
	{
		// Todo
		return false;
	}

	bool isClose(Node other)
	{
		float position = this->distanceSquared(other);
		float orientation = abs(theta - other.theta);

		return position <= tolerance_radius && orientation <= tolerance_angle;
	}
};

double control_frequency = 10.0;

class PathPlanning
{
  public:
	ros::NodeHandle nh;
	ros::Publisher paths_pub;
	ros::Subscriber robot_position;

	//Initialisation
	float x0, y0, theta0, position_updated;

	PathPlanning(ros::NodeHandle nh, ros::Publisher paths_pub)
	{

		this->nh = nh;
		this->paths_pub = paths_pub;
		robot_position = nh.subscribe("/deadreckogning/Pos", 1000, &PathPlanning::getPositionCallBack, this);

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
		float delta, delta_resolution, angular_velocity, angle_diff_tol, cost_to_come;
		std::vector<Node> successors;

		delta_resolution = pi / 4.0;

		angle_diff_tol = 1e-1;

		for (float delta = -pi; delta < pi; delta += delta_resolution)
		{
			float x, y, theta, path_cost, dt, t;
			x = node.x;
			y = node.y;
			theta = node.theta;

			angular_velocity = -pi / 4.0 * (float)((theta - delta > 0) - (theta - delta < -1.0e-3));
			//ROS_INFO("x: %f    y %f   th %f   delta %f   angular_velocity %f    path_cost: %f   t: %f", (float)((theta - delta > 0) - (theta - delta < 0)), y, theta, delta, angular_velocity, (float)(theta - delta > 0), theta - delta);

			t = 0.0;
			dt = 0.04;

			std::vector<float> path_x, path_y;
			path_cost = 0.0f;
			cost_to_come = node.cost_to_come;

			//while (std::abs(delta - theta) > angle_diff_tol)
			//for (int i = 0; i < 50; i++)
			while (t < 1)
			{
				//ROS_INFO("x: %f    y %f   th %f   delta %f   angular_velocity %f    path_cost: %f   t: %f", x, y, theta, delta, angular_velocity, path_cost, t);
				float diff = std::abs(delta - theta);
				x += cos(theta) * dt;
				y += sin(theta) * dt;
				theta += angular_velocity * dt;

				t += dt;
				path_x.push_back(x);
				path_y.push_back(y);

				path_cost += sqrt(pow(cos(theta), 2.0) + pow(sin(theta), 2.0));
			}
			//ROS_INFO("delta: %f    theta %f ", delta, theta);

			cost_to_come += path_cost;

			Node successor_node = Node(x, y, theta, angular_velocity, t, path_x, path_y, path_cost, cost_to_come);

			successors.push_back(successor_node);
		}

		return successors;
	}

	robo7_msgs::paths getPath()
	{
		float randomInt, goal_radius_tolerance;
		std::vector<float> path_x, path_y;
		std::vector<Node> successors, alive_nodes, dead_nodes;
		robo7_msgs::path path_msg;
		robo7_msgs::paths paths_msg;

		goal_radius_tolerance = .3;

		if (position_updated)
		{

			Node node_start = Node(x0, y0, pi / 2, 0.0f, 0.0f, path_x, path_y, 0.0f, 0.0f);
			Node node_target = Node(xt, yt, 0.0f, 0.0f, 0.0f, path_x, path_y, 0.0f, 0.0f);

			alive_nodes.push_back(node_start);

			while (!alive_nodes.empty())
			{
				//ROS_INFO("size alive: %d", (int)alive_nodes.size());
				//ROS_INFO("size dead: %d", (int)dead_nodes.size());
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

				if (node_current.distanceSquared(node_target) < goal_radius_tolerance)
				{
					ROS_INFO("Goal reached");
				}

				alive_nodes.erase(alive_nodes.begin() + node_current.alive_node_index);
				dead_nodes.push_back(node_current);

				successors = getSuccessorNodes(node_current);

				for (int i = 0; i < successors.size(); i++)
				{
					Node &node_successor = successors[i];
					node_successor.parent = &node_current;
					//node_successor.successor = node_current;

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
								alive_nodes.erase(alive_nodes.begin() + node_current.alive_node_index);
								dead_nodes.push_back(node_successor);
								break;
							}
						}

						if (!match)
						{
							alive_nodes.push_back(node_successor);
							path_msg.path_x = node_successor.path_x;
							path_msg.path_y = node_successor.path_y;
							paths_msg.paths.push_back(path_msg);
							paths_pub.publish(paths_msg);
						}
					}
				}
			}

			return paths_msg;
		}

		std::vector<float> empty_path_x, empty_path_y;

		path_msg.path_x = empty_path_x;
		path_msg.path_y = empty_path_y;
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



	PathPlanning path_planning = PathPlanning(nh, paths_pub);

	ros::Rate loop_rate(control_frequency);

	robo7_msgs::paths paths_msg;

	while (nh.ok())
	{
		paths_msg = path_planning.getPath();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
