#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/home_position.hpp>

#include "uav_msgs/msg/event_uav.hpp"
#include <uav_msgs/msg/target_global.hpp>
#include <uav_msgs/msg/user_cmd.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <cmath>
#include <string>
#include <type_traits>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// Variables
float lat_cur = NAN, lon_cur = NAN, alt_cur = NAN; // vị trí GPS hiện tại
float lat_hold = NAN, lon_hold = NAN, alt_hold = NAN, yaw_hold = NAN; // vị trí GPS hold khi vào POSITION HOLD mode
std::vector<double> tgt_lat, tgt_lon, tgt_alt; // danh sách WAYPOINTS GPS mục tiêu (WGS84)
double home_lat = NAN, home_lon = NAN, home_alt = NAN; // snapshot khi ARM
double px4_home_lat, px4_home_lon, px4_home_alt; // vi tri home cua px4 khi arm

bool home_locked = false; //Home truoc khi bay?
bool armed = false;

//NED
float x_cur = 0.f, y_cur = 0.f, z_cur = 0.f;       // local NED hiện tại
float vx = 0.0, vy = 0.0, vz = 0;
// float x_sp = 0.0, y_sp = 0.0, z_sp = 0.0; 
// float x_des = 0.0, y_des = 0.0, z_des = 0.0;

float roll_des = 0.0, pitch_des = 0.0, yaw_des = 0.0; // roll, pitch setpoint
float roll = 0.0, pitch = 0.0, yaw = 0.0; // roll, pitch, yaw estimate

// State machine
enum class State { NONE, IDLE, TAKEOFF, YAW_ALIGN, CRUISE, RTL, POS_MODE };
State state_ = State::NONE;

//Security buttons
bool user_cmd_arm = false;
bool user_cmd_rtl = false;
bool user_cmd_pause = false;
bool user_cmd_continue = false;
bool user_cmd_set_home = false;

bool user_cmd_yaw_align = false;
float user_cmd_yaw_angle = 0.0;

bool land_detect = false;

/////PAYLOAD
int target_index = 0;
int target_reach = NAN; // --- IGNORE ---
bool payload_release = false; // --- IGNORE ---

// yaw face-to-target 
inline float bearing_gps_rad(double lat_cur_deg, double lon_cur_deg,
                             double lat_sp_deg,  double lon_sp_deg)
{
    const double deg2rad = M_PI / 180.0;
    double lat1 = lat_cur_deg * deg2rad;
    double lon1 = lon_cur_deg * deg2rad;
    double lat2 = lat_sp_deg  * deg2rad;
    double lon2 = lon_sp_deg  * deg2rad;

    double dLon = lon2 - lon1;

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);

    double bearing = atan2(y, x);  // rad
    return bearing; // kết quả nằm trong [-pi, pi]
}
// Hàm tính khoảng cách Haversine giữa hai điểm tọa độ lat/lon
// lat1, lon1, lat2, lon2: Tọa độ tính bằng độ
// return: Khoảng cách tính bằng mét
double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0; // Bán kính Trái Đất tính bằng mét
    const double PI = 3.14159265358979323846;

    // Chuyển đổi từ độ sang radian
    double lat1_rad = lat1 * PI / 180.0;
    double lon1_rad = lon1 * PI / 180.0;
    double lat2_rad = lat2 * PI / 180.0;
    double lon2_rad = lon2 * PI / 180.0;

    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;

    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon / 2.0) * sin(dlon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return R * c;
}
// Template function to get the message version suffix
// The correct message version is directly inferred from the message defintion
// trait: có T::MESSAGE_VERSION hay không
template<typename T, typename = void>
struct has_message_version : std::false_type {};
template<typename T>
struct has_message_version<T, std::void_t<decltype(T::MESSAGE_VERSION)>> : std::true_type {};
template<class T>
std::string getMessageNameVersion() {
    if constexpr (has_message_version<T>::value) {
        if constexpr (T::MESSAGE_VERSION == 0) {
            return "";
        } else {
            return std::string("_v") + std::to_string(T::MESSAGE_VERSION);
        }
    } else {
        // message không versioned (px4_msgs cũ): không thêm suffix
        return "";
    }
}
//Subscription
class Subscription : public rclcpp::Node
{
    public:
        Subscription() : Node("subscription")
        {
            vehicle_local_position_subscriber();
            vehicle_global_position_subscriber();
            vehicle_attitude_subscriber();
            vehicle_status_subscriber();
            vehicle_land_detected_subscriber();
            vehicle_home_position_subscriber();

            uav_target_global_subscriber();
            uav_user_cmd_subscriber();
        }
        private:
            rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
            rclcpp::Subscription<uav_msgs::msg::TargetGlobal>::SharedPtr uav_target_global_subscriber_;         
            rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_subscriber_;
            rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
            rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
            rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_subscriber_;
            rclcpp::Subscription<uav_msgs::msg::UserCmd>::SharedPtr uav_user_cmd_subscriber_;
            rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr vehicle_home_position_subscriber_;

            void vehicle_local_position_subscriber();
            void uav_target_global_subscriber();
            void vehicle_global_position_subscriber();
            void vehicle_attitude_subscriber();
            void vehicle_status_subscriber();
            void vehicle_land_detected_subscriber();
            void uav_user_cmd_subscriber();
            void vehicle_home_position_subscriber();
};

void Subscription::vehicle_local_position_subscriber()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
    const std::string sub_topic = "/fmu/out/vehicle_local_position" + getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>();
    vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        sub_topic, qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
            x_cur = msg->x; y_cur = msg->y; z_cur = msg->z;
            vx = msg->vx; vy = msg->vy; vz = msg->vz;
        });
}

void Subscription::vehicle_land_detected_subscriber()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

     // Use template function to define the correct topics automatically
    const std::string sub_topic = "/fmu/out/vehicle_land_detected" + getMessageNameVersion<px4_msgs::msg::VehicleLandDetected>();

    vehicle_land_detected_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
        sub_topic, qos,
        [this](const px4_msgs::msg::VehicleLandDetected::UniquePtr msg) {
            land_detect = msg->ground_contact;
        });
}

void Subscription::uav_target_global_subscriber()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    uav_target_global_subscriber_ = this->create_subscription<uav_msgs::msg::TargetGlobal>(
        "/target_global", qos,
        [this](const uav_msgs::msg::TargetGlobal::UniquePtr msg) {
            tgt_lat = msg->lat;
            tgt_lon = msg->lon;
            tgt_alt = msg->alt_m;

            target_index = 0; //reset target khi nhan waypoint moi tu user

            state_ = State::IDLE; // reset state machine để thực hiện nhiệm vụ mới

            for (size_t i = 0; i < tgt_lat.size(); i++) {
                RCLCPP_INFO(this->get_logger(),
                    "  WP %ld: lat=%.7f lon=%.7f alt=%.2f",
                    i + 1, tgt_lat[i], tgt_lon[i], tgt_alt[i]);
            };
            });
}

void Subscription::uav_user_cmd_subscriber()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    uav_user_cmd_subscriber_ = this->create_subscription<uav_msgs::msg::UserCmd>(
        "/user_cmd", qos,
        [this](const uav_msgs::msg::UserCmd::UniquePtr msg) {
            user_cmd_arm = msg->arm;
            user_cmd_rtl = msg->rth;
            user_cmd_pause = msg->pause;
            user_cmd_continue = msg->resume;
            user_cmd_set_home = msg->set_home;
            user_cmd_yaw_align = msg->yaw_align;
            user_cmd_yaw_angle = msg->yaw_align_angle;
            //pause mode
            if (user_cmd_pause) 
            {
                lat_hold = lat_cur;
                lon_hold = lon_cur;
                alt_hold = alt_cur;
                yaw_hold = yaw;
                state_ = State::POS_MODE;
                RCLCPP_INFO(this->get_logger(), "User command: Mission Pause");
            }
            //resume
            if (user_cmd_continue && state_ == State::POS_MODE) 
            {
                state_ = State::IDLE;
                RCLCPP_INFO(this->get_logger(), "Continue mission");
            }      
            //RTL
            if (user_cmd_rtl) {
                state_ = State::RTL;
                RCLCPP_INFO(this->get_logger(), "User command: RTL");
            }
            if((user_cmd_yaw_align) && (state_ == State::NONE)) 
            {
                RCLCPP_INFO(this->get_logger(),"start yaw_align");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"cannot yaw_align");
                user_cmd_yaw_align = false;
            }
            

        });
}

void Subscription::vehicle_global_position_subscriber()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

     // Use template function to define the correct topics automatically
    const std::string sub_topic = "/fmu/out/vehicle_global_position" + getMessageNameVersion<px4_msgs::msg::VehicleGlobalPosition>();

    vehicle_global_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        sub_topic, qos,
        [this](const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
            lat_cur = msg->lat; lon_cur = msg->lon; alt_cur = msg->alt;
            // RCLCPP_INFO(this->get_logger(), "Received Vehicle Global Position: lat=%.6f, lon=%.6f, alt=%.3f", lat, lon, alt);
        });
}

void Subscription::vehicle_attitude_subscriber()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

     // Use template function to define the correct topics automatically
    const std::string sub_topic = "/fmu/out/vehicle_attitude" + getMessageNameVersion<px4_msgs::msg::VehicleAttitude>();

    vehicle_attitude_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        sub_topic, qos,
        [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
            double w = msg->q[0];
            double x = msg->q[1];
            double y = msg->q[2];
            double z = msg->q[3];

            //Quaternion to Euler angles
            roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
            pitch = asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
            yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        });
}

void Subscription::vehicle_status_subscriber()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

     // Use template function to define the correct topics automatically
    const std::string sub_topic = "/fmu/out/vehicle_status" + getMessageNameVersion<px4_msgs::msg::VehicleStatus>();

    vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        sub_topic, qos,
        [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {

            // logic : khi chuyển sang armed lần đầu -> chốt home.
                armed = (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED);
        });
}
void Subscription::vehicle_home_position_subscriber()
{
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

     // Use template function to define the correct topics automatically
    const std::string sub_topic = "/fmu/out/home_position" + getMessageNameVersion<px4_msgs::msg::HomePosition>();

    vehicle_home_position_subscriber_ = this->create_subscription<px4_msgs::msg::HomePosition>(
        sub_topic, qos,
        [this](const px4_msgs::msg::HomePosition::UniquePtr msg) {
            px4_home_lat = msg->lat;
            px4_home_lon = msg->lon;
            px4_home_alt = msg->alt;
        });
}

class OffboardControl : public rclcpp::Node
{
    public:

        OffboardControl() : Node("offboard_control")
        {
            publisher();
            event_update();
            mission();
            auto timer_callback = [this]() -> void {
                position_setpoint();
            };
            timer_ = this->create_wall_timer(100ms, timer_callback);
        }
    private:
        void publisher();
        void event_update();
        void mission();
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

        void take_off(float alt_takeoff);
        void waypoint(float lat_wp, float lon_wp, float alt_wp, float yaw_wp);
        void yaw_adjust(float lat_wp, float lon_wp, float alt_wp, float yaw_wp);
        void alt_adjust(float lat_wp, float lon_wp, float alt_wp, float yaw_wp);

        void return_to_launch();
        void position_setpoint();

        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_; 
        rclcpp::Publisher<uav_msgs::msg::EventUav>::SharedPtr event_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;  

        float yaw_ = 0.f; // rad
        float lat_yaw_tgt = 0.f, lon_yaw_tgt = 0.f, yaw_face = 0.f; // yaw nhìn về target
        float alt_takeoff = 0.f; // m
};


void OffboardControl::position_setpoint()
{

    if(!land_detect && !home_locked)
    {
        home_lat = px4_home_lat;
        home_lon = px4_home_lon;
        home_alt = px4_home_alt;

        home_locked = true;
    }
    // "Home set" button
    if(user_cmd_set_home)
    {
        if(land_detect)
        {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1, 0); // set home to current position

            home_lat = px4_home_lat;
            home_lon = px4_home_lon;
            home_alt = px4_home_alt;

            home_locked = true;

            RCLCPP_INFO(this->get_logger(),
                "Home locked @ User Cmd Set Home: lat=%.7f lon=%.7f alt=%.2f",
                home_lat, home_lon, home_alt);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "CANNOT SET HOME, NOT LAND DETECTED");
        }
        user_cmd_set_home = false;
    }
        
    switch (state_) {
        case State::NONE:{
            if ((user_cmd_arm) && (!armed)) {
                
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
                RCLCPP_INFO(this->get_logger(), "User command: ARM");

                home_lat = lat_cur;
                home_lon = lon_cur;
                home_alt = alt_cur;

                home_locked = true;

                RCLCPP_INFO(this->get_logger(),
                    "Home locked @ ARM: lat=%.7f lon=%.7f alt=%.2f",
                    home_lat, home_lon, home_alt);

                user_cmd_arm = false;
            }
            //USER_CMD_YAW_ALIGN
            if(user_cmd_yaw_align)
            {
                if(!land_detect)
                {
                    yaw_adjust(lat_cur, lon_cur, alt_cur, user_cmd_yaw_angle * M_PI/180);

                    if (std::fabs(user_cmd_yaw_angle - yaw) < 10*M_PI/180.0f) // 10 độ
                    {
                        RCLCPP_INFO(this->get_logger(),"yaw align done");
                        user_cmd_yaw_align = false;
                    }
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),"cannot yaw_align");
                    user_cmd_yaw_align = false;

                }
            }
            break;
            }
        case State::IDLE:{

            if ((user_cmd_arm) && (!armed)) {
                
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
                RCLCPP_INFO(this->get_logger(), "User command: ARM");
                user_cmd_arm = false;
            }

            if (home_locked == true )
            {

                alt_takeoff = home_alt + tgt_alt[target_index]; // alt relative to home
                lat_yaw_tgt = lat_cur;
                lon_yaw_tgt = lon_cur;
                yaw_ = yaw;

                state_ = State::TAKEOFF;      
                RCLCPP_INFO(this->get_logger(),"ready height adjust");
            }

            break;
            }
        case State::TAKEOFF: {
            //if disarm -> arm
            if(!armed)
            {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
            }
                alt_adjust(lat_yaw_tgt, lon_yaw_tgt, alt_takeoff, yaw_);
                // RCLCPP_INFO(this->get_logger(),"height adjust");

            if(std::fabs(alt_cur - alt_takeoff ) < 0.5f)
            {
                // Face-to-target yaw từ vị trí hiện tại -> goal
                yaw_face = bearing_gps_rad(lat_cur, lon_cur, tgt_lat[target_index], tgt_lon[target_index]);
                RCLCPP_INFO(this->get_logger(),"ready yaw_align");

                state_ = State::YAW_ALIGN;
            }
            break;
        }

        case State::YAW_ALIGN: {

            yaw_adjust(lat_yaw_tgt, lon_yaw_tgt, alt_takeoff, yaw_face);

            if (std::fabs(yaw_face - yaw) < 10*M_PI/180.0f) // 10 độ
            {
                state_ = State::CRUISE;
                RCLCPP_INFO(this->get_logger(),"ready cruise");
            }
                // RCLCPP_INFO(this->get_logger(),"yaw_align, yaw_face: %.2f, yaw_cur: %.2f", yaw_face*180.0f/M_PI, yaw*180.0f/M_PI);
            break;
        }

        case State::CRUISE: {

            waypoint(tgt_lat[target_index], tgt_lon[target_index], alt_takeoff, yaw_face);
                // RCLCPP_INFO(this->get_logger(),"send waypoint ");

            double distance_to_goal = haversine_distance(lat_cur, lon_cur, tgt_lat[target_index], tgt_lon[target_index]);

            if (distance_to_goal < 0.7) // trong bán kính 1m
                {
                    payload_release = true;
                    RCLCPP_INFO(this->get_logger(), "Reached target %d", target_index + 1);
                    event_update();


                if (tgt_lat.empty()) 
                {
                    state_ = State::NONE; // Không có đích nào để đi
                }
                // Nếu index hiện tại đã lớn hơn hoặc bằng index cuối cùng
                // (Index cuối cùng là size() - 1)
                else if (target_index >= tgt_lat.size() - 1)
                {
                    // Đây là điểm đích cuối cùng rồi, hoàn thành
                    state_ = State::NONE;
                }
                else
                {
                    // Di chuyển đến điểm đích tiếp theo
                    target_index++;

                    state_ = State::IDLE;
                }
                }
            break;
        }
        case State::RTL: {
            payload_release = false;
            mission();
            if (armed) {
                // RTL về home, thong so chinh trong para
                return_to_launch();
                // RCLCPP_INFO(this->get_logger(), "Returning to Launch (RTL)");
            } else {
                RCLCPP_INFO(this->get_logger(), "Vehicle is disarmed, cannot RTL");
                //set postion mode
        	    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3); 
                state_ = State::NONE;
            }
            break;
        }
        case State::POS_MODE: {
                yaw_adjust(lat_hold, lon_hold, alt_hold, yaw_hold);
                RCLCPP_INFO(this->get_logger(), "Holding position");
            break;
        }
    }
}

void OffboardControl::publisher()
{
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    event_publisher_ = this->create_publisher<uav_msgs::msg::EventUav>("/event_uav", 10);

}
void OffboardControl::mission()
{
    uav_msgs::msg::EventUav event_msg{};
    event_msg.payload_action = payload_release;    
    event_publisher_->publish(event_msg);
}  
void OffboardControl::event_update()
{
    uav_msgs::msg::EventUav event_msg{};
    event_msg.payload_action = payload_release;    
    event_msg.target_reach =  static_cast<uint32_t>(target_index + 1);
    event_publisher_->publish(event_msg);
}

void OffboardControl::take_off(float alt_takeoff)
{
    VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
    msg.param1 = 0.0f; // minimum pitch (if airspeed sensor present), desired pitch without sensor
    msg.param2 = 0.0f; // empty
    msg.param3 = NAN; // yaw angle (if magnetometer present), ignored without magnetometer
    msg.param4 = NAN;  // latitude, ignored
    msg.param5 = NAN;  // longitude, ignored
    msg.param6 = NAN;  // altitude (AMSL, WGS84 or local, depending on frame), in meters
    msg.param7 = alt_takeoff; // altitude relative to home position, in meters
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_publisher_->publish(msg);
}

void OffboardControl::return_to_launch()
{
    VehicleCommand cmd{};
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    cmd.command = VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
    cmd.param1 = 0.0f; // empty
    cmd.param2 = 0.0f; // empty
    cmd.param3 = 0.0f; // empty
    cmd.param4 = NAN;  // empty
    cmd.param5 = NAN;  // empty
    cmd.param6 = NAN;  // empty
    cmd.param7 = NAN;  // empty
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    vehicle_command_publisher_->publish(cmd);
}

void OffboardControl::waypoint(float lat_wp, float lon_wp, float alt_wp, float yaw_wp)
{
    VehicleCommand cmd{};
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    cmd.command = VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
    cmd.param1 = -1.0f; // ground speed (m/s), -1 to use current speed
    // param2 = bitmask
    // bit 0 (1<<0): đặt yaw
    // bit 1 (1<<1): yaw tính theo hướng tới WP (đặt 0 nếu muốn dùng param4)
    // -> dùng yaw tuyệt đối: set bit0 = 1, bit1 = 0 => mask = 1
    cmd.param2 = 1.0f;
    // param3 = yaw rate (không dùng) -> NAN
    cmd.param3 = NAN;
    // param4 = yaw mong muốn (PX4 thường hiểu rad; nếu bạn đang đi MAVLink thuần là deg)
    cmd.param4 = NAN ;
    // param5-7 = lat, lon, alt (deg, deg, m AMSL)
    cmd.param5 = lat_wp;
    cmd.param6 = lon_wp;
    cmd.param7 = alt_wp;

    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    vehicle_command_publisher_->publish(cmd);
}

void OffboardControl::yaw_adjust(float lat_wp, float lon_wp, float alt_wp, float yaw_wp)
{
    VehicleCommand cmd{};
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    cmd.command = VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
    cmd.param1 = -1.0f; 
    cmd.param2 = 0.0f;
    cmd.param3 = NAN;
    cmd.param4 = yaw_wp ;
    cmd.param5 = lat_wp;
    cmd.param6 = lon_wp;
    cmd.param7 = alt_wp;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    vehicle_command_publisher_->publish(cmd);
}

void OffboardControl::alt_adjust(float lat_wp, float lon_wp, float alt_wp, float yaw_wp)
{
    VehicleCommand cmd{};
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    cmd.command = VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
    cmd.param1 = -1.0f; 
    cmd.param2 = 0.0f;
    cmd.param3 = NAN;
    cmd.param4 = yaw_wp ;
    cmd.param5 = lat_wp;
    cmd.param6 = lon_wp;
    cmd.param7 = alt_wp;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    vehicle_command_publisher_->publish(cmd);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_publisher_->publish(msg);
}


int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    auto subscriber = std::make_shared<Subscription>();
    auto offboard_control = std::make_shared<OffboardControl>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(subscriber);
    executor.add_node(offboard_control);

    executor.spin();

	rclcpp::shutdown();
	return 0;
}
