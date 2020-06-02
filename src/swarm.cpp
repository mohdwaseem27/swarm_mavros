#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


ros::Publisher local_pos_pub0;
ros::Publisher local_pos_pub1;
ros::Publisher local_pos_pub2;

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
   
    
    geometry_msgs::PoseStamped pose;
 void pubpoints(int x, int y, int z, int uav){
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
   
    if (uav==0){
       local_pos_pub0.publish(pose);;
     }

    else if (uav==1){    
      local_pos_pub1.publish(pose);;
     }

    else if (uav==2){
    local_pos_pub2.publish(pose);;
     }      
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm");
    ros::NodeHandle nh;

    ros::Subscriber state_sub0 = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, state_cb);

    ros::Subscriber state_sub1 = nh.subscribe<mavros_msgs::State>
            ("/uav1/mavros/state", 10, state_cb);

    ros::Subscriber state_sub2 = nh.subscribe<mavros_msgs::State>
            ("/uav2/mavros/state", 10, state_cb);

    local_pos_pub0 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);

    local_pos_pub1 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav1/mavros/setpoint_position/local", 10);

    local_pos_pub2 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav2/mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client0 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");

    ros::ServiceClient arming_client1 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav1/mavros/cmd/arming");


    ros::ServiceClient arming_client2 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav2/mavros/cmd/arming");

    ros::ServiceClient land_client0 = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/uav0/mavros/cmd/land");

    ros::ServiceClient land_client1 = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/uav1/mavros/cmd/land");

    ros::ServiceClient land_client2 = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/uav2/mavros/cmd/land");

    ros::ServiceClient set_mode_client0 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");

    ros::ServiceClient set_mode_client1 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav1/mavros/set_mode");

    ros::ServiceClient set_mode_client2 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav2/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z =2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose);
	local_pos_pub1.publish(pose);
	local_pos_pub2.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land;
    land.request.yaw = 0;
    land.request.latitude = 0;
    land.request.longitude = 0;
    land.request.altitude = 0;

    ros::Time last_request = ros::Time::now();
    // change to offboard mode and arm
    while(ros::ok() && !current_state.armed){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client0.call(offb_set_mode) &&
                set_mode_client1.call(offb_set_mode) &&
                set_mode_client2.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client0.call(arm_cmd) && 
		    arming_client1.call(arm_cmd) &&
		    arming_client2.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub0.publish(pose);
        local_pos_pub1.publish(pose);
        local_pos_pub2.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

      ROS_INFO("takingoff");
      ROS_INFO("marking");
       for(int i = 10; ros::ok() && i > 0; --i){
          pubpoints(0, 0, 2, 0);
          ros::spinOnce();
          rate.sleep();

          pubpoints(0, 0, 2, 1);
          ros::spinOnce();
          rate.sleep();

          pubpoints(0, 0, 2, 2);
          ros::spinOnce();
          rate.sleep();
        }
  

       for(int i = 30; ros::ok() && i > 0; --i){
          pubpoints(1, 1, 1, 0);
          ros::spinOnce();
          rate.sleep();

          pubpoints(0, 1, 2.5, 1);
          ros::spinOnce();
          rate.sleep();

          pubpoints(1, 0, 3.25, 2);
          ros::spinOnce();
          rate.sleep();
        }

      
      ROS_INFO("planning for the path I");  
       for(int i = 30; ros::ok() && i > 0; --i){
          pubpoints(3, 1, 1, 0);
          ros::spinOnce();
          rate.sleep();

          pubpoints(2, 1, 2.5, 1);
          ros::spinOnce();
          rate.sleep();

          pubpoints(3, 0, 3.25, 2);
          ros::spinOnce();
          rate.sleep();
        }

       for(int i = 10; ros::ok() && i > 0; --i){
          pubpoints(3, 0, 2.5, 0);
          ros::spinOnce();
          rate.sleep();

          pubpoints(2, 1, 2.5, 1);
          ros::spinOnce();
          rate.sleep();

          pubpoints(3, 1, 2.25, 2);
          ros::spinOnce();
          rate.sleep();
        }

 
       for(int i = 20; ros::ok() && i > 0; --i){
          pubpoints(3, 1, 3.25, 0);
          ros::spinOnce();
          rate.sleep();

          pubpoints(2, 1, 2.5, 1);
          ros::spinOnce();
          rate.sleep();

          pubpoints(3, 0, 1, 2);
          ros::spinOnce();
          rate.sleep();
        }

      ROS_INFO("planning for the path III");  
       for(int i = 30; ros::ok() && i > 0; --i){
          pubpoints(2, 0, 2.5, 0);
          ros::spinOnce();
          rate.sleep();

          pubpoints(2, 1, 2.5, 1);
          ros::spinOnce();
          rate.sleep();

          pubpoints(2, 1, 2.5, 2);
          ros::spinOnce();
          rate.sleep();
        }

       for(int i = 30; ros::ok() && i > 0; --i){
          pubpoints(5, 0, 2.5, 0);
          ros::spinOnce();
          rate.sleep();

          pubpoints(5, 1, 2.5, 1);
          ros::spinOnce();
          rate.sleep();

          pubpoints(5, 1, 2.5, 2);
          ros::spinOnce();
          rate.sleep();
        }

      ROS_INFO("retreating initial position");
       for(int i = 0; ros::ok() && i < 75; ++i){
          pubpoints(0, 0, 2, 0);
          pubpoints(0, 0, 2, 1);
          pubpoints(0, 0, 2, 2);
          ros::spinOnce();
          rate.sleep();
        }
    

    ROS_INFO("landing");
    while (!(land_client0.call(land) &&
            land_client1.call(land) &&
            land_client2.call(land) &&
            land.response.success)){
      local_pos_pub0.publish(pose);
      local_pos_pub1.publish(pose);
      local_pos_pub2.publish(pose);
      ROS_INFO("landing");
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
