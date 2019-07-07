#include <phoxi_ocs/fixed_phoxi_tf_broadcaster.h>

// Node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fixed_phoxi_tf_broadcaster");
  ros::NodeHandle nh;

  FixedPhoxiTFBroadcaster broadcaster(nh);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    broadcaster.broadcast();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
