#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv) {	
ros::init(argc, argv, "tf_listener");

	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");

	std::string from_frame_, to_frame_;

    from_frame_ = "/world";
    to_frame_ = "/camera0";
    
    nhp.param("from_frame", from_frame_, from_frame_);
    nhp.param("to_frame", to_frame_, to_frame_);

	ros::Publisher tf_pub = nh.advertise<geometry_msgs::TransformStamped>("/rovio/T_CW", 10);

	tf::TransformListener Lnr_CW;

	ros::Rate rate(10.0);

	while(nh.ok()) {
		tf::StampedTransform tf_CW;
		try {
			Lnr_CW.lookupTransform(from_frame_, to_frame_, ros::Time(0), tf_CW);
		} catch (tf::TransformException &ex) {
		    ROS_ERROR("%s",ex.what());
		    ros::Duration(1.0).sleep();
		    continue;
	    }
	    // tf::StampedTransform T_CW = tf_CO * tf_OW;

	    geometry_msgs::TransformStamped tf_msg;

	    tf::transformStampedTFToMsg(tf_CW, tf_msg);
	    tf_pub.publish(tf_msg);
	    rate.sleep();
	}
	return 0;
}