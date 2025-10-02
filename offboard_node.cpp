#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath> 
#include <algorithm>

// تعريف ثابت لسرعة إرسال نقاط الهدف (20 هرتز)
#define RATE 20.0 

// ثوابت الحركة - تم التعديل هنا لزيادة المقياس
#define TAKEOFF_Z 6.0           // ارتفاع الطيران (متر)
#define RADIUS 15.0             // **مقاس كبير: 15 متر نصف قطر**
#define ANGULAR_SPEED 0.3       // **سرعة زاوية أبطأ** للمسار الكبير (0.3 راديان/ثانية)

// تعريف مراحل المهمة
#define PHASE_TAKEOFF 1
#define PHASE_FIGURE8 2
#define PHASE_LAND 3
#define PHASE_COMPLETE 4

// ************************ متغيرات الحالة العالمية ************************
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_local_pose;
ros::Time mission_start_time;
ros::Time phase_start_time;
int current_phase = PHASE_TAKEOFF; 

// ************************ دوال Callbacks ************************

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_pose = *msg;
}

// ************************ الدالة الرئيسية ************************
int main(int argc, char **argv)
{
    ros::init(argc, argv, "large_figure8_node");
    ros::NodeHandle nh;

    // ************ الناشرون والمشتركون ************
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(RATE);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("MAVROS connected. Starting Large Single Figure-8 Mission.");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = TAKEOFF_Z; 

    // تهيئة Offboard والتسليح 
    for(int i = 0; ros::ok() && i < 100; i++){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    mission_start_time = ros::Time::now(); 
    phase_start_time = ros::Time::now(); 

    // ************ حساب المدة المطلوبة لدورة كاملة (2*PI/السرعة الزاوية) ************
    const double required_figure8_duration = (2.0 * M_PI) / ANGULAR_SPEED; 
    ROS_INFO("Calculated duration for one full Figure-8 loop: %.2f seconds.", required_figure8_duration);

    // ************ الحلقة الرئيسية للتحكم ************
    while(ros::ok()){
        // 1. طلب وضع Offboard والتسليح باستمرار
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                phase_start_time = ros::Time::now(); 
            }
            last_request = ros::Time::now();
        } else {
            if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // 2. منطق مراحل المهمة
        if (current_state.armed && current_state.mode == "OFFBOARD")
        {
            double time_in_phase = (ros::Time::now() - phase_start_time).toSec();
            
            switch (current_phase)
            {
                case PHASE_TAKEOFF: 
                    pose.pose.position.x = 0.0;
                    pose.pose.position.y = 0.0;
                    pose.pose.position.z = TAKEOFF_Z;
                    
                    if (time_in_phase >= 15.0) 
                    {
                        current_phase = PHASE_FIGURE8; 
                        ROS_INFO("Phase 1 Complete: Reached %.1f m. Starting single Figure-8 loop.", TAKEOFF_Z);
                        phase_start_time = ros::Time::now(); 
                    }
                    break;

                case PHASE_FIGURE8: 
                    { 
                        double figure8_time = time_in_phase;
                        
                        // معادلات الشكل 8 (Lemniscate of Gerono)
                        double angle = ANGULAR_SPEED * figure8_time;
                        
                        // X = R * sin(angle)
                        pose.pose.position.x = RADIUS * std::sin(angle);
                        
                        // Y = R * sin(angle) * cos(angle)
                        pose.pose.position.y = RADIUS * std::sin(angle) * std::cos(angle); 
                        
                        // Z ثابت (6 متر)
                        pose.pose.position.z = TAKEOFF_Z;

                        // **شرط الخروج: إكمال دورة واحدة فقط**
                        if (figure8_time >= required_figure8_duration) {
                            current_phase = PHASE_LAND; 
                            ROS_INFO("Phase 2 Complete: Single Figure-8 loop finished. Initiating LAND.");
                            // إعادة تعيين الهدف إلى المركز قبل الهبوط
                            pose.pose.position.x = 0.0;
                            pose.pose.position.y = 0.0;
                        } else {
                            ROS_INFO_THROTTLE(5.0, "FIGURE-8: X=%.1f, Y=%.1f. Remaining time for one loop: %.1f s", 
                                              pose.pose.position.x, pose.pose.position.y, required_figure8_duration - figure8_time);
                        }
                    }
                    break;
                
                case PHASE_LAND: 
                {
                    mavros_msgs::SetMode land_mode;
                    land_mode.request.custom_mode = "AUTO.LAND";
                    if(set_mode_client.call(land_mode) && land_mode.response.mode_sent){
                        ROS_INFO("Phase 3: LAND mode initiated. Mission Complete.");
                    }
                    current_phase = PHASE_COMPLETE;
                    break;
                }
                
                default:
                    break;
            }
        }
        
       
        if (current_phase < PHASE_LAND)
        {
            local_pos_pub.publish(pose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}