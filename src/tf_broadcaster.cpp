#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv) {	
ros::init(argc, argv, "tf_broadcaster");

	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");

	std::string from_frame_, to_frame_, body_frame_;

    from_frame_ = "/cam0";
    to_frame_ = "/body";
    // body_frame_ = "/body";

    nhp.param("from_frame", from_frame_, from_frame_);
    nhp.param("to_frame", to_frame_, to_frame_);

	ros::Publisher tf_pub = nh.advertise<geometry_msgs::TransformStamped>("/T_BC", 10);

	// tf::TransformListener Lnr_CW;

	ros::Rate rate(10.0);

	XmlRpc::XmlRpcValue T_body_cam;

	if (!nhp.getParam("T_marker_cam", T_body_cam) )
		ROS_ERROR("Failed to read T_marker_cam on param server");

    tf::Vector3 org_CW;
    tf::Quaternion rot_CW, rot_BC;
    
    tf::Matrix3x3 rotMat(
		(tfScalar)T_body_cam[0][0], (tfScalar)T_body_cam[0][1], (tfScalar)T_body_cam[0][2], 
		(tfScalar)T_body_cam[1][0], (tfScalar)T_body_cam[1][1], (tfScalar)T_body_cam[1][2], 
		(tfScalar)T_body_cam[2][0], (tfScalar)T_body_cam[2][1], (tfScalar)T_body_cam[2][2]);
    rotMat.getRotation(rot_BC);

    tf::Vector3 org_BC( T_body_cam[0][3], T_body_cam[1][3], T_body_cam[2][3] );
    tf::Transform tf_BC (rot_BC, org_BC);

    tf::TransformBroadcaster br;

	while(nh.ok()) {
		// tf::StampedTransform tf_CW;
		// try {
		// 	Lnr_CW.lookupTransform(from_frame_, to_frame_, ros::Time(0), tf_CW);
		// } catch (tf::TransformException &ex) {
		//     ROS_ERROR("%s",ex.what());
		//     ros::Duration(1.0).sleep();
		//     continue;
	 //    }

	    // org_CW = tf_CW.getOrigin();
	    // rot_CW = tf_CW.getRotation();

	    // tf_BW = tf_BC * tf::Transform(rot_CW, org_CW);
	    tf::StampedTransform T_BC( tf_BC, ros::Time::now(), from_frame_, to_frame_ );

	    geometry_msgs::TransformStamped tf_msg;

	    tf::transformStampedTFToMsg(T_BC, tf_msg);
	    tf_pub.publish(tf_msg);
	    br.sendTransform(T_BC);
	    rate.sleep();
	}
	return 0;
}