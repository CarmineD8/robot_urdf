#include <ros/ros.h>
//MOVE IT
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{

ros::init(argc, argv, "custom_planning");
ros::NodeHandle nh;
ros::AsyncSpinner spinner(1);
spinner.start();
sleep(2.0);

moveit::planning_interface::MoveGroupInterface group("arm");
group.setEndEffectorLink("arm_link_04");
group.setPoseReferenceFrame("base_link");
group.setPlannerId("RRTstar");
group.setNumPlanningAttempts(10);
group.setPlanningTime(10.0);
group.allowReplanning(true);
group.setGoalJointTolerance(0.0001);
group.setGoalPositionTolerance(0.0001);
group.setGoalOrientationTolerance(0.001);
int choice;

while (ros::ok()){

std::cout << "Press 1 to reach the zero position, press 2 to reach a different position: ";
std::cin >> choice;

if (choice == 1){

	group.setNamedTarget("zero");
	group.move();  
	sleep(5.0);
}

else if (choice == 2){

	geometry_msgs::Pose target_pose1;
	//This should be a valid pose 
	target_pose1.orientation.w = 1.0;
	target_pose1.orientation.x = 0.0;
	target_pose1.orientation.y = 0.0;
	target_pose1.orientation.z = 0.0;
	target_pose1.position.x =  0.1;
	target_pose1.position.y =  0.0;
	target_pose1.position.z =  0.2;


	group.setStartStateToCurrentState();
	group.setApproximateJointValueTarget(target_pose1, "arm_link_04");

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	group.plan(my_plan);
	group.execute(my_plan);

	sleep(5.0);
}
}

ros::shutdown();

return 0;
}
