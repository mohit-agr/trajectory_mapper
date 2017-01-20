#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_cameras");

	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");

	std::string from_frame_, to_frame_;

    from_frame_ = "/cam0";
    to_frame_ = "/cam1";
    
    nhp.param("from_frame", from_frame_, from_frame_);
    nhp.param("to_frame", to_frame_, to_frame_);

	ros::Publisher tf_pub = nh.advertise<geometry_msgs::TransformStamped>("/T_C1C0", 10);

	tf::TransformListener Lnr_CC;

	ros::Rate rate(10.0);

	while(nh.ok()) {
		tf::StampedTransform tf_CC;
		try {
			Lnr_CC.lookupTransform(from_frame_, to_frame_, ros::Time(0), tf_CC);
			// Lnr_OW.lookupTransform("world", "odom", ros::Time(0), tf_OW);
		} catch (tf::TransformException &ex) {
		    ROS_ERROR("%s",ex.what());
		    ros::Duration(1.0).sleep();
		    continue;
	    }
	    // tf::StampedTransform T_CW = tf_CO * tf_OW;

	    geometry_msgs::TransformStamped tf_msg;

	    tf::transformStampedTFToMsg(tf_CC, tf_msg);
	    tf_pub.publish(tf_msg);
	    rate.sleep();
	}
	return 0;
}