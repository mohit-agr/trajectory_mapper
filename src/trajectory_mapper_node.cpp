#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>


class pathMapNode {
  ros::NodeHandle nh_;
  ros::Subscriber transform_sub;
  tf::TransformBroadcaster path_broadcaster;
  ros::Publisher path_pub;

  std::vector<geometry_msgs::PoseStamped> poses;
    
public:
  pathMapNode()
  {
    transform_sub = nh_.subscribe("tf_msg", 10, &pathMapNode::tfCallback, this);
    path_pub = nh_.advertise<nav_msgs::Path>("path", 50);
  }  

  void tfCallback(const geometry_msgs::TransformStamped& msg)
  {
    tf::StampedTransform transform;
    tf::transformStampedMsgToTF(msg, transform);
    // tf::Stamped<tf::Pose> pose( tf::Transform(transform));
    // geometry_msgs::PoseStamped pose_msg (msg);
    // tf::poseStampedTFToMsg(pose, pose_msg);

    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.stamp = msg.header.stamp;
    pose_stamped.header.frame_id = "world";

    pose_stamped.pose.position.x = transform.getOrigin().getX();
    pose_stamped.pose.position.y = transform.getOrigin().getY();
    pose_stamped.pose.position.z = transform.getOrigin().getZ();

    pose_stamped.pose.orientation.x = transform.getRotation().getX();
    pose_stamped.pose.orientation.y = transform.getRotation().getY();
    pose_stamped.pose.orientation.z = transform.getRotation().getZ();
    pose_stamped.pose.orientation.w = transform.getRotation().getW();

    // pose_publisher.publish(pose_stamped);

    static uint count=0;
    // std::cout << "Saving poses" << std::endl;
    poses.push_back(pose_stamped);
    count++;
    // std::cout << "Saving poses" << std::endl;
    nav_msgs::Path path;
    path.header.stamp = msg.header.stamp;
    path.header.frame_id = "world";

    if (count >= 5)
    {
      // nav_msgs::Path path;

      path.poses = poses;
      poses.empty(); count = 0;
      path_pub.publish(path);
    }
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_mapper_node");


  // ros::Time current_time, last_time;
  // current_time = ros::Time::now();
  // last_time = ros::Time::now();

  // ros::Rate r(1.0);
  pathMapNode pmn;
  ros::spin();
    // current_time = ros::Time::now();



    //compute odometry in a typical way given the velocities of the robot

    // last_time = current_time;
    // r.sleep();
}