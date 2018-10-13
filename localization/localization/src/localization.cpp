//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <phidgets/motor_encoder.h>
#include <std_msgs/Float32.h>


// Control @ 10 Hz
double control_frequency = 100.0;

class deadReckogning
{
public:
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber encoder_Left;
  ros::Subscriber encoder_Right;
  ros::Subscriber estimated_L_speed;
  ros::Subscriber estimated_R_speed;
  ros::Publisher robot_position1;
  ros::Publisher robot_position2;

  deadReckogning()
  {
    n = ros::NodeHandle("~");

    //Initialisation of the position
    nh.param<float>("/deadreckogning/initial_x_pos", x_pos, 0);
    nh.param<float>("/deadreckogning/initial_y_pos", y_pos, 0);
    nh.param<float>("/deadreckogning/initial_z_angle", z_angle, 0);

    //Definition of the adjustment parameters
    nh.param<float>("/deadreckogning/linear_adjustment", linear_adjustment, 0);
    nh.param<float>("/deadreckogning/angular_adjustment", angular_adjustment, 0);

    x2_pos = x_pos;
    y2_pos = y_pos;
    z2_angle = z_angle;

    wheel_radius = 97.6/2000.0; //m
    wheel_distance = 217.3/1000.0; //m
    tics_per_rev = 897.96;
    pi = 3.14159265358979323846;

    //Initialisation
    encoder_R = 0;
    encoder_L = 0;
    count_L = 0;
    count_R = 0;
    prev_count_L = 0;
    prev_count_R = 0;

    //Non-linearised model
    arc_radius = 0;
    arc_angle = 0;
    arc_distance = 0;

    //Other parameters
    Dt = 1/control_frequency; //ms - time between two consecutive iterations

    encoder_Left = n.subscribe("/l_motor/encoder", 10, &deadReckogning::encoder_L_callBack, this);
    encoder_Right = n.subscribe("/r_motor/encoder", 10, &deadReckogning::encoder_R_callBack, this);
    robot_position1 = n.advertise<geometry_msgs::Twist>("Pos", 10);
    robot_position2 = n.advertise<geometry_msgs::Twist>("Pos2", 10);

  }

  void encoder_L_callBack(const phidgets::motor_encoder::ConstPtr &msg)
  {
      count_L = msg->count;
  }

  void encoder_R_callBack(const phidgets::motor_encoder::ConstPtr &msg)
  {
      count_R = msg->count;
  }

  void updatePosition(){
    //Generate the future published twist msg
    geometry_msgs::Twist twist_msg;
    geometry_msgs::Twist twist_msg2;

    //Update the differents count changes
    encoder_L = count_L - prev_count_L;
    encoder_R = count_R - prev_count_R;

    prev_count_L = count_L;
    prev_count_R = count_R;

    //Guess the values of both wheel's angular speeds with signs
    om_L = angular_motor_distance(encoder_L);
    om_R = angular_motor_distance(encoder_R);

    // Compute the linear and angular velocities
    ang_dis = angular_distance_linearised(om_L, -om_R);
    lin_dis = linear_distance_linearised(om_L, -om_R);


    //Compute the non linear distances and angles
    arc_angle = angular_distance_linearised(om_L, -om_R);
    arc_distance = linear_distance_linearised(om_L, -om_R);
    if(arc_angle!=0){
      arc_radius = arc_distance/arc_angle;
      x2_pos += 2*arc_radius*sin(arc_angle/2)*(cos(arc_angle/2)*cos(z2_angle)+sin(arc_angle/2)*sin(z2_angle));
      y2_pos += 2*arc_radius*sin(arc_angle/2)*(cos(arc_angle/2)*sin(z2_angle)+sin(arc_angle/2)*cos(z2_angle));
    }
    else{
      x2_pos += (arc_distance * cos(z2_angle));
      y2_pos += (arc_distance * sin(z2_angle));
    }
    z2_angle += arc_angle;
    z2_angle = wrapAngle(z2_angle);

    //Adjustment
    x2_pos = linear_adjustment + x2_pos;
    y2_pos = linear_adjustment + y2_pos;
    z2_angle = angular_adjustment + z2_angle;

    twist_msg2.linear.x = x2_pos;
    twist_msg2.linear.y = y2_pos;
    twist_msg2.linear.z = 0;

    twist_msg2.angular.x = 0;
    twist_msg2.angular.y = 0;
    twist_msg2.angular.z = z2_angle;


    //Compute the linear distances and angles of the robot
    x_pos += (lin_dis * cos(z_angle)) * (1 + linear_adjustment);
    y_pos += (lin_dis * sin(z_angle)) * (1 + linear_adjustment);
    z_angle += ang_dis * (1 + angular_adjustment);
    z_angle = wrapAngle(z_angle);

    //Adjustment
    // x_pos = linear_adjustment + x_pos;
    // y_pos = linear_adjustment + y_pos;
    // z_angle = angular_adjustment + z_angle;

    twist_msg.linear.x = x_pos;
    twist_msg.linear.y = y_pos;
    twist_msg.linear.z = 0;

    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = z_angle;

    ROS_INFO("Publishing in deadreckogning with x: %f", x_pos);

    //Send the datas
    robot_position1.publish(twist_msg);
    robot_position2.publish(twist_msg2);
  }

private:
  //Position update
  float x_pos;
  float y_pos;
  float z_angle;
  float x2_pos;
  float y2_pos;
  float z2_angle;

  //All the physical dimensions of the robot
  float wheel_radius;
  float wheel_distance;
  float tics_per_rev;
  float pi;

  //Other parameters
  float Dt; //ms - time between two consecutive iterations

  //encoders values
  int encoder_L;
  int encoder_R;
  //Counts
  int count_L;
  int count_R;
  //Prev counts
  int prev_count_L;
  int prev_count_R;
  //Guess the values of both wheel's angular speeds with signs
  float om_L;
  float om_R;
  //Compute the linear and angular velocities
  float ang_dis;
  float lin_dis;

  //Non linearised model
  float arc_radius;
  float arc_angle;
  float arc_distance;

  //Adjustment parameters
  float linear_adjustment;
  float angular_adjustment;

  //Other useful function
  int sgn(int v)
  {
    if (v < 0) return -1;
    if (v > 0) return 1;
    return 0;
  }


  float angular_motor_distance(int encod)
  {
    return ((2*pi*encod)/(tics_per_rev));
  }

  float linear_distance_linearised(float left_wheel_speed, float right_wheel_speed)
  {
    return wheel_radius*(right_wheel_speed + left_wheel_speed)/2;
  }

  float angular_distance_linearised(float left_wheel_speed, float right_wheel_speed)
  {
    return wheel_radius*(right_wheel_speed - left_wheel_speed)/wheel_distance;
  }

  float wrapAngle( double angle )
  {
    float twoPi = 2.0 * pi;
    return angle - twoPi * floor( angle / twoPi );
  }
};


class findWallPoints
{

}


class robot_point
{
public:
  float x;
  float y;
  float theta;
  float weight;

  robot_point(float x_init, float y_init, float theta_init, float weight_init)
  {
    x = x_init;
    y = y_init;
    theta = theta_init;
    weight = weight_init;
  }

  void updatePoint(float x_new, float y_new, float theta_new, float weight_new)
  {
    x = x_new;
    y = y_new;
    theta = theta_new;
    weight = weight_new;
  }
};


class cloud_point
{
public:
  std::vector<robot_point> _points_weighted;



};






















class computeWallPointsDiscretisation
{
public:
  std::vector<float> X_wall_coordinates;
  std::vector<float> Y_wall_coordinates;

  computeWallPointsDiscretisation(string map_txt_file)

  ifstream map_fs; map_fs.open(map_txt_file.c_str());
  if (!map_fs.is_open()){
      ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
      return -1;
  }

  string line;
  int wall_id = 0;
  while (getline(map_fs, line)){

      if (line[0] == '#') {
          // comment -> skip
          continue;
      }

      max_num = std::numeric_limits<double>::max();
      x1= max_num;
      x2= max_num;
      y1= max_num;
      y2= max_num;

      std::istringstream line_stream(line);

      line_stream >> x1 >> y1 >> x2 >> y2;

      if ((x1 == max_num) || ( x2 == max_num) || (y1 == max_num) || (y2 == max_num)){
          ROS_WARN("Segment error. Skipping line: %s",line.c_str());
      }


      // angle and distance
      N_step = floor( sqrt(pow(x1-x2,2) + pow(y1-y2,2))/discretisation_step );
      x_step = (x1-x2)/N_step; y_step = (y1-y2)/N_step;
      for(int i=0; i<N_step; i++)
      {
        X_wall_coordinates.push_back(x1 + i*x_step);
        Y_wall_coordinates.push_back(y1 + i*y_step);
      }
  }

private:
  //Discretisation of the map into (x,y) coordinates
  float discretisation_step;
  int N_step;

  //Wall parameters
  float max_num;
  float x1; float y1; float x2; float y2;
  float x_step;
  float y_step;
};





class localization
{
public:

  localization()
  {
    string _map_file;
    string _map_frame = "/map";
    string _map_topic = "/maze_map";
    n.param<string>("map_file", _map_file, "maze_map.txt");

    computeWallPointsDiscretisation Wall_points(_map_file);

    //Initialisation of the position
    nh.param<float>("/deadreckogning/initial_x_pos", x_pos, 0);
    nh.param<float>("/deadreckogning/initial_y_pos", y_pos, 0);
    nh.param<float>("/deadreckogning/initial_z_angle", z_angle, 0);

    //Definition of the adjustment parameters
    nh.param<float>("/deadreckogning/linear_adjustment", linear_adjustment, 0);
    nh.param<float>("/deadreckogning/angular_adjustment", angular_adjustment, 0);

    wheel_radius = 97.6/2000.0; //m
    wheel_distance = 217.3/1000.0; //m
    tics_per_rev = 897.96;
    pi = 3.14159265358979323846;

    //Initialisation
    encoder_R = 0;
    encoder_L = 0;
    count_L = 0;
    count_R = 0;
    prev_count_L = 0;
    prev_count_R = 0;

    //Non-linearised model
    arc_radius = 0;
    arc_angle = 0;
    arc_distance = 0;

    //Other parameters
    Dt = 1/control_frequency; //ms - time between two consecutive iterations

    encoder_Left = n.subscribe("/l_motor/encoder", 10, &deadReckogning::encoder_L_callBack, this);
    encoder_Right = n.subscribe("/r_motor/encoder", 10, &deadReckogning::encoder_R_callBack, this);
    robot_position1 = n.advertise<geometry_msgs::Twist>("Pos", 10);
    robot_position2 = n.advertise<geometry_msgs::Twist>("Pos2", 10);

  }

  void ICP()
  {


  }

private:
  //Localization main parameters
  float current_x;
  float current_y;

  //Lidar function parameters
};







int main(int argc, char **argv)
{
    ros::init(argc, argv, "deadreckogning");

    deadReckogning dead_reckogning;

    ros::Rate loop_rate(control_frequency);

    while(dead_reckogning.n.ok())
    {
        dead_reckogning.updatePosition();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
