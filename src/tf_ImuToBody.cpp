#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class RovioImuToBody {
	ros::NodeHandle nh;

	ros::Subscriber imu_sub, at_sub;
	ros::Publisher body_pub;

	tf::StampedTransform rovio_imu;
	tf::Transform body_imu;

	std::string parent_frame_, mid1_frame_, mid2_frame_, child_frame_, uav_;

public:
	RovioImuToBody() : 
	parent_frame_("cam3"), 
	mid1_frame_("imu"),
	mid2_frame_("cam0"),
	child_frame_("body") {
		imu_sub = nh.subscribe("/rovio/extrinsics0", 10, &RovioImuToBody::tfCallback, this);
		at_sub = nh.subscribe("/tf_cam0", 1, &RovioImuToBody::aprilCb, this);
		body_pub = nh.advertise<geometry_msgs::TransformStamped>("/body_pose", 1);

		ros::NodeHandle nhp("~");
		nhp.param("parent_frame", parent_frame_, parent_frame_);
		nhp.param("mid1_frame", mid1_frame_, mid1_frame_);
		nhp.param("mid2_frame", mid2_frame_, mid2_frame_);
		nhp.param("child_frame", child_frame_, child_frame_);
		nhp.param("uav_", uav_, uav_);
		bool loadSuccess = true;
		XmlRpc::XmlRpcValue tf_mid1_parent, tf_mid2_mid1, tf_child_mid2;

		loadSuccess &= nhp.getParam("T_" + mid1_frame_ + "_" + parent_frame_, tf_mid1_parent);
		loadSuccess &= nhp.getParam("T_" + mid2_frame_ + "_" + mid1_frame_, tf_mid2_mid1);
		loadSuccess &= nhp.getParam("T_" + child_frame_ + "_" + mid2_frame_, tf_child_mid2);

		tf::Quaternion rot_m1p, rot_m2m1, rot_cm2;

		tf::Matrix3x3 rotMat(
			(tfScalar)tf_mid1_parent[0][0], (tfScalar)tf_mid1_parent[0][1], (tfScalar)tf_mid1_parent[0][2], 
			(tfScalar)tf_mid1_parent[1][0], (tfScalar)tf_mid1_parent[1][1], (tfScalar)tf_mid1_parent[1][2], 
			(tfScalar)tf_mid1_parent[2][0], (tfScalar)tf_mid1_parent[2][1], (tfScalar)tf_mid1_parent[2][2]);
	    rotMat.getRotation(rot_m1p);
	    tf::Vector3 org_m1p(tf_mid1_parent[0][3], tf_mid1_parent[1][3], tf_mid1_parent[2][3]);

		rotMat = tf::Matrix3x3(
			(tfScalar)tf_mid2_mid1[0][0], (tfScalar)tf_mid2_mid1[0][1], (tfScalar)tf_mid2_mid1[0][2], 
			(tfScalar)tf_mid2_mid1[1][0], (tfScalar)tf_mid2_mid1[1][1], (tfScalar)tf_mid2_mid1[1][2], 
			(tfScalar)tf_mid2_mid1[2][0], (tfScalar)tf_mid2_mid1[2][1], (tfScalar)tf_mid2_mid1[2][2]);
	    rotMat.getRotation(rot_m2m1);
	    tf::Vector3 org_m2m1(tf_mid2_mid1[0][3], tf_mid2_mid1[1][3], tf_mid2_mid1[2][3]);

	    rotMat = tf::Matrix3x3(
			(tfScalar)tf_child_mid2[0][0], (tfScalar)tf_child_mid2[0][1], (tfScalar)tf_child_mid2[0][2], 
			(tfScalar)tf_child_mid2[1][0], (tfScalar)tf_child_mid2[1][1], (tfScalar)tf_child_mid2[1][2], 
			(tfScalar)tf_child_mid2[2][0], (tfScalar)tf_child_mid2[2][1], (tfScalar)tf_child_mid2[2][2]);
	    rotMat.getRotation(rot_cm2);

	    tf::Vector3 org_cm2( tf_child_mid2[0][3], tf_child_mid2[1][3], tf_child_mid2[2][3] );

	    body_imu = tf::Transform(rot_cm2, org_cm2) * (tf::Transform(rot_m2m1, org_m2m1) * tf::Transform(rot_m1p, org_m1p) );
	}

	void tfCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
		geometry_msgs::Pose pose_msg = msg.pose.pose;
		tf::Transform Tf;
		tf::poseMsgToTF(pose_msg, Tf);

		tf::StampedTransform stf(body_imu * Tf, msg.header.stamp, "/origin", uav_ + "/" + child_frame_);
		geometry_msgs::TransformStamped tf_msg;
		tf::transformStampedTFToMsg(stf, tf_msg);
		body_pub.publish(tf_msg);
	}

	void aprilCb(const geometry_msgs::TransformStamped& msg) {
		// geometry_msgs::Pose pose_msg = msg.pose.pose;
		tf::StampedTransform Tf;
		tf::transformStampedMsgToTF(msg, Tf);

		tf::StampedTransform stf(body_imu * Tf, msg.header.stamp, "/origin", uav_ + "/" + child_frame_);
		geometry_msgs::TransformStamped tf_msg;
		tf::transformStampedTFToMsg(stf, tf_msg);
		body_pub.publish(tf_msg);
	}
};

int main(int argc, char** argv) {	
	ros::init(argc, argv, "tf_ImuToBody");

	RovioImuToBody itb;

	ros::spin();
	return 0;
}
