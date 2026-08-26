#include <ros/ros.h>
#include <Eigen/Dense>
#include <fstream>
#include <regex>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/Px4ctrlState.h>
#include <mavros_msgs/RCIn.h>

using namespace std;

bool px4_is_auto_hover = false;

uint8_t px4_historical_status = quadrotor_msgs::Px4ctrlState::MANUAL_CTRL;

// button-press state for RC channel 8 (now a momentary switch)
// start as "not pressed" because initial state is UP (unpressed)
bool rc_button_prev = false;

// timestamp of last effective button press (used for 1‑second debounce)
ros::Time last_valid_press = ros::Time(0);

// 8通道按键触发的指令序列:
//   手动模式(auto_planning=0): 第1次=起飞, 第2次=开始航点飞行, 第3次=降落
//   自动模式(auto_planning=1): 第1次=起飞, 航点飞行与降落由程序自动完成
enum BUTTON_STEP {STEP_TAKEOFF = 0, STEP_MOVE = 1, STEP_LAND = 2};
BUTTON_STEP rc_step = STEP_TAKEOFF;

ros::Publisher point_pub;
ros::Publisher yaw_pub;
ros::Publisher takeoff_land_pub;
ros::Publisher startcommand_pub;
ros::Subscriber odom_sub;
ros::Subscriber startcommand_sub;
ros::Subscriber rc_sub;
ros::Subscriber px4_mode_sub;
ros::Timer timer;

// 添加新的订阅
ros::Subscriber goal_modified_sub_;
std::map<int, Eigen::Vector3d> modified_goals_;


enum RC_EIGHT_STATE
{
    RC_EIGHT_UP = 990,
    RC_EIGHT_MIDDLE = 1499,
    RC_EIGHT_DOWN = 1990
};

// 定义结构体表示每个点的数据
struct pytStr {
    double x;
    double y;
    double z;
    double yaw = -100;
    double time;
};

Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
double odom_yaw_ = 0.0;                               // current yaw angle from odometry
geometry_msgs::PoseStamped goal;  //目标点
vector<pytStr> pytVector;  //存放yaml读取的点
vector<pytStr> start_pytVector;  //存放yaml读取的点
int counts = 0;  //正在规划的点位次序
double distance_ = 10.0;  //规划的点到当前点的距离
double next_distance;
quadrotor_msgs::PositionCommand cmd_yaw;
bool trigger = false;
int flag_start_plan;
int auto_planning;
bool enable_yaw_align;
double yaw_align_thresh;

// Parse comma-separated numbers inside a single [...] chunk, e.g. "10.0, 0.0, 0.8"
static vector<double> parseBracketContent(const string& inner)
{
    vector<double> values;
    std::stringstream ss(inner);
    string token;
    while (std::getline(ss, token, ','))
    {
        // trim whitespace
        size_t s = token.find_first_not_of(" \t\r\n");
        size_t e = token.find_last_not_of(" \t\r\n");
        if (s != string::npos)
            values.push_back(std::stod(token.substr(s, e - s + 1)));
    }
    return values;
}
void readpyt(string file_path)
{
    if(file_path.empty())
    {
        ROS_ERROR("The YAML file path is empty,Failed to load pyt!");
        return;
    }

    std::ifstream file(file_path);
    if (!file.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_path.c_str());
        return;
    }

    bool in_points = false;
    std::regex bracket_re(R"(\[([^\[\]][^\]]*)\])");
    string line;

    while (std::getline(file, line))
    {
        if (line.find("points:") != string::npos && line.find("back_points") == string::npos)
            in_points = true;
        else if (line.find("back_points:") != string::npos)
            in_points = false;  // 忽略旧配置中遗留的返航段
        else if (line.find("test") != string::npos)
            in_points = false;  // 忽略旧 test 段

        if (!in_points) continue;

        size_t comment_pos = line.find('#');
        if (comment_pos != string::npos)
            line = line.substr(0, comment_pos);

        auto it = std::sregex_iterator(line.begin(), line.end(), bracket_re);
        auto end = std::sregex_iterator();

        for (; it != end; ++it)
        {
            vector<double> vals = parseBracketContent((*it)[1].str());
            if (vals.empty()) continue;

            if (vals.size() < 3 || vals.size() > 5)
            {
                ROS_ERROR("Point format error: expected 3-5 elements, got %zu", vals.size());
                return;
            }
            pytStr pyt_;
            pyt_.x = vals[0]; pyt_.y = vals[1]; pyt_.z = vals[2];
            pyt_.yaw = (vals.size() >= 4) ? vals[3] : -100;
            pyt_.time = (vals.size() >= 5) ? vals[4] : 0;
            start_pytVector.push_back(pyt_);
        }
    }

    if (start_pytVector.empty())
    {
        ROS_ERROR("No points loaded. Check the file format (expect a 'points:' section).");
        return;
    }

    ROS_INFO("Loaded waypoints:");
    for (size_t i = 0; i < start_pytVector.size(); i++)
    {
        ROS_INFO("  point %zu: [%.2f, %.2f, %.2f, yaw=%.1f, time=%.1f]",
                 i+1, start_pytVector[i].x, start_pytVector[i].y, start_pytVector[i].z,
                 start_pytVector[i].yaw, start_pytVector[i].time);
    }
    ROS_INFO("Arrival threshold: %.2f m", next_distance);
}


void px4_mode_cb(const quadrotor_msgs::Px4ctrlState::ConstPtr &msg)
{
    // 自动模式: 起飞完成(AUTO_TAKEOFF -> AUTO_HOVER)后自动开始航点飞行
    if (px4_historical_status == quadrotor_msgs::Px4ctrlState::AUTO_TAKEOFF &&
        msg->state == quadrotor_msgs::Px4ctrlState::AUTO_HOVER &&
        auto_planning == 1)
    {
        geometry_msgs::PoseStamped startcommand_msg;
        startcommand_msg.header.stamp = ros::Time::now();
        startcommand_pub.publish(startcommand_msg);
        cout << "middle --> up" << endl;
    }

    // 降落完成(AUTO_LAND -> MANUAL_CTRL)后复位按键状态机, 支持下一轮飞行
    if (px4_historical_status == quadrotor_msgs::Px4ctrlState::AUTO_LAND &&
        msg->state == quadrotor_msgs::Px4ctrlState::MANUAL_CTRL)
    {
        rc_step = STEP_TAKEOFF;
        ROS_INFO("[multipoint] Land finished, button sequence reset to TAKEOFF");
    }

    px4_historical_status = msg->state;

    // 检查 px4ctrl 是否处于 AUTO_HOVER 状态
    px4_is_auto_hover = (msg->state == quadrotor_msgs::Px4ctrlState::AUTO_HOVER);
}

void odom_goal_cb(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // extract yaw from quaternion
    double w = msg->pose.pose.orientation.w;
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    odom_yaw_ = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}


// 在计算距离之前，获取当前目标点（优先使用修改后的）
Eigen::Vector3d getCurrentTargetPoint(int idx)
{
    auto it = modified_goals_.find(idx);
    if (it != modified_goals_.end()) {
        return it->second;
    }
    return Eigen::Vector3d(pytVector[idx].x, pytVector[idx].y, pytVector[idx].z);
}


void Point_send(const ros::TimerEvent& event)
{
    if(!trigger){
        return;
    }
    else if(pytVector.size() == 0)   //判断是否还有点
    {
        trigger = false;
        return;
    }

    if(counts == 0){
        // yaw pre-alignment: rotate to face target direction before flying
        if (enable_yaw_align)
        {
            double target_dir = atan2(pytVector[counts].y - odom_pos_(1), pytVector[counts].x - odom_pos_(0));
            double yaw_diff = fabs(target_dir - odom_yaw_);
            if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;

            if (yaw_diff > yaw_align_thresh)
            {
                ROS_INFO("[multipoint] Yaw pre-alignment triggered: current=%.2f, target_dir=%.2f, diff=%.1f deg",
                         odom_yaw_, target_dir, yaw_diff * 180.0 / M_PI);

                timer.stop();
                ros::Rate rate(20);
                while (ros::ok())
                {
                    // send yaw+position to traj_server, which handles smooth rotation
                    cmd_yaw.position.x = odom_pos_(0);
                    cmd_yaw.position.y = odom_pos_(1);
                    cmd_yaw.position.z = odom_pos_(2);
                    cmd_yaw.yaw = target_dir;
                    yaw_pub.publish(cmd_yaw);

                    double cur = fabs(target_dir - odom_yaw_);
                    if (cur > M_PI) cur = 2 * M_PI - cur;
                    if (cur < 0.05) break;

                    rate.sleep();
                    ros::spinOnce();
                }
                timer.start();
                ROS_INFO("[multipoint] Yaw pre-alignment complete");
            }
        }

        // 使用修改后的目标点
        Eigen::Vector3d first_target = getCurrentTargetPoint(0);
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = first_target.x();
        goal.pose.position.y = first_target.y();
        goal.pose.position.z = first_target.z();
        point_pub.publish(goal);
        // Clear pre-alignment yaw when waypoint has no explicit yaw (yaw=-100),
        // otherwise traj_server arrival alignment would rotate at the waypoint.
        if (enable_yaw_align && pytVector[0].yaw == -100)
        {
          cmd_yaw.yaw = -100;
          yaw_pub.publish(cmd_yaw);
        }
        ROS_INFO("Publish first waypoint [%d/%zu]: [%.2f, %.2f, %.2f, yaw=%.1f, time=%.1f]",
                 counts+1, start_pytVector.size(),
                 goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                 pytVector[counts].yaw, pytVector[counts].time);
        counts++;
    }

    int counts_pre = counts - 1;
    // Eigen::Vector3d point_cur{pytVector[counts_pre].x, pytVector[counts_pre].y, pytVector[counts_pre].z};
    Eigen::Vector3d point_cur = getCurrentTargetPoint(counts_pre);
    distance_ = (point_cur - odom_pos_).norm();
    // ROS_INFO("Distance to next point: %f", distance_);

    if (distance_ < next_distance && counts < pytVector.size())   //到达目标点并且还有下一个点
    {
        if(pytVector[counts_pre].time > 0){
            timer.stop();
            ROS_INFO("Waiting %.1fs at waypoint %d", pytVector[counts_pre].time, counts_pre + 1);

            double time_wait = pytVector[counts_pre].time;
            ros::Duration(time_wait).sleep();
            ros::spinOnce();
            timer.start();
        }

        // yaw pre-alignment: rotate to face next target direction before sending goal
        if (enable_yaw_align)
        {
            double target_dir = atan2(pytVector[counts].y - odom_pos_(1), pytVector[counts].x - odom_pos_(0));
            double yaw_diff = fabs(target_dir - odom_yaw_);
            if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;

            if (yaw_diff > yaw_align_thresh)
            {
                ROS_INFO("[multipoint] Yaw pre-alignment triggered: current=%.2f, target_dir=%.2f, diff=%.1f deg",
                         odom_yaw_, target_dir, yaw_diff * 180.0 / M_PI);

                timer.stop();
                ros::Rate rate(20);
                while (ros::ok())
                {
                    // send yaw+position to traj_server, which handles smooth rotation
                    cmd_yaw.position.x = odom_pos_(0);
                    cmd_yaw.position.y = odom_pos_(1);
                    cmd_yaw.position.z = odom_pos_(2);
                    cmd_yaw.yaw = target_dir;
                    yaw_pub.publish(cmd_yaw);

                    double cur = fabs(target_dir - odom_yaw_);
                    if (cur > M_PI) cur = 2 * M_PI - cur;
                    if (cur < 0.05) break;

                    rate.sleep();
                    ros::spinOnce();
                }
                timer.start();
                ROS_INFO("[multipoint] Yaw pre-alignment complete");
            }
        }

        // 使用修改后的目标点
        Eigen::Vector3d next_target = getCurrentTargetPoint(counts);

        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = next_target.x();
        goal.pose.position.y = next_target.y();
        goal.pose.position.z = next_target.z();
        point_pub.publish(goal);
        // Clear pre-alignment yaw when waypoint has no explicit yaw (yaw=-100)
        if (enable_yaw_align && pytVector[counts].yaw == -100)
        {
          cmd_yaw.yaw = -100;
          yaw_pub.publish(cmd_yaw);
        }
        ROS_INFO("Publish next waypoint [%d/%zu]: [%.2f, %.2f, %.2f, yaw=%.1f, time=%.1f]",
                 counts+1, start_pytVector.size(),
                 goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                 pytVector[counts].yaw, pytVector[counts].time);
        counts++;
    }

    if (distance_ > next_distance && counts_pre < pytVector.size())
    {
        if(pytVector[counts_pre].yaw != -100)
        {
            cmd_yaw.position.x = odom_pos_(0);
            cmd_yaw.position.y = odom_pos_(1);
            cmd_yaw.position.z = odom_pos_(2);
            cmd_yaw.yaw = pytVector[counts_pre].yaw;
            yaw_pub.publish(cmd_yaw);
        }
    }

    // 自动模式: 到达最后一个点并自动降落
    if (counts_pre == pytVector.size() - 1 && distance_ < next_distance && auto_planning == 1 && px4_is_auto_hover)
    {
        ROS_INFO("Reached the final waypoint!");
        trigger = false;  // 停止规划

        quadrotor_msgs::TakeoffLand takeoff_msg;
        takeoff_msg.takeoff_land_cmd = 2;
        takeoff_land_pub.publish(takeoff_msg);
        cout << "middle --> down" << endl;
    }
}

void startplan_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(start_pytVector.size() == 0)
    {
        ROS_ERROR("No pyt loaded!");
        return;
    }

    pytVector.assign(start_pytVector.begin(), start_pytVector.end());
    counts = 0;
    trigger = true;
    // P0 修复: 清除上一轮飞行遗留的推离目标点, 防止旧修改点污染新一轮的到达判定
    modified_goals_.clear();
    ROS_INFO("Get start trigger.");
}

void rc_cb(mavros_msgs::RCInConstPtr pMsg)
{
    // channel 8 作为自复位按钮: 按下为 DOWN, 松开为 UP
    bool pressed = (pMsg->channels[7] >= RC_EIGHT_DOWN);

    // 只在上升沿(false->true)响应, 忽略持续按住
    if (pressed && !rc_button_prev)
    {
        // 消抖: 两次有效按键之间至少间隔 1 秒
        ros::Time now = ros::Time::now();
        if ((now - last_valid_press).toSec() < 1.0)
        {

            ROS_INFO("rc button ignored due to debounce");
        }
        else
        {
            switch (rc_step)
            {
                case STEP_TAKEOFF:
                    // 第1次按下: 起飞 (仅当未处于自动悬停时)
                    if (!px4_is_auto_hover)
                    {
                        quadrotor_msgs::TakeoffLand takeoff_msg;
                        takeoff_msg.takeoff_land_cmd = 1;
                        takeoff_land_pub.publish(takeoff_msg);
                        ROS_INFO("down -> takeoff");
                        rc_step = STEP_MOVE;
                    }
                    break;

                case STEP_MOVE:
                    // 第2次按下(仅手动模式): 起飞悬停后开始航点飞行
                    // 自动模式(auto_planning==1)下航点飞行自动触发, 按键不响应
                    if (px4_is_auto_hover && auto_planning == 0)
                    {
                        geometry_msgs::PoseStamped startcommand_msg;
                        startcommand_msg.header.stamp = ros::Time::now();
                        startcommand_pub.publish(startcommand_msg);
                        ROS_INFO("takeoff -> move");
                        rc_step = STEP_LAND;
                    }
                    break;

                case STEP_LAND:
                    // 第3次按下(仅手动模式): 悬停时降落
                    if (px4_is_auto_hover)
                    {
                        quadrotor_msgs::TakeoffLand takeoff_msg;
                        takeoff_msg.takeoff_land_cmd = 2;
                        takeoff_land_pub.publish(takeoff_msg);
                        ROS_INFO("move -> land");
                        rc_step = STEP_TAKEOFF; // 复位状态机, 支持下一轮飞行
                    }
                    break;

                default:
                    break;
            }
        }

        last_valid_press = now;
    }

    rc_button_prev = pressed;
}


void goalModifiedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    int current_wp_idx = counts - 1;

    if (current_wp_idx < 0 || current_wp_idx >= (int)pytVector.size()) {
        ROS_WARN("[multipoint] Invalid current waypoint index: %d, counts=%d, pytVector.size=%zu",
                 current_wp_idx, counts, pytVector.size());
        return;
    }

    // P1 修复: 校验 seq 与当前航点是否匹配, 丢弃迟到的旧航点修改消息
    // planner 端 seq = wpt_id_ (每收到一个 /goal 先 ++), 处理第 counts-1 个航点时 seq == counts
    if ((int)msg->header.seq != counts)
    {
        ROS_WARN("[multipoint] /goal_modified seq=%u mismatch, expected=%d (current waypoint %d). Discarded.",
                 msg->header.seq, counts, current_wp_idx);
        return;
    }

    Eigen::Vector3d pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Vector3d original_pos(pytVector[current_wp_idx].x,
                                  pytVector[current_wp_idx].y,
                                  pytVector[current_wp_idx].z);

    // 存储修改后的目标点
    modified_goals_[current_wp_idx] = pos;

    ROS_INFO("[multipoint] Goal modified for current waypoint %d: original(%.2f,%.2f,%.2f) -> modified(%.2f,%.2f,%.2f)",
             current_wp_idx,
             original_pos.x(), original_pos.y(), original_pos.z(),
             pos.x(), pos.y(), pos.z());

    // 关键：重新计算距离，让到达判断能立即生效
    distance_ = (pos - odom_pos_).norm();
    ROS_INFO("[multipoint] Updated distance to modified target: %.2f (threshold=%.2f)",
             distance_, next_distance);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "multipointplan_node");
    ros::NodeHandle nh;

    string yaml_path;
    if (!nh.getParam("/multipointplan/yaml_path", yaml_path) ||
        !nh.getParam("/multipointplan/next_distance", next_distance) ||
        !nh.getParam("/multipointplan/start_plan", flag_start_plan) ||
        !nh.getParam("/multipointplan/auto_planning", auto_planning))
    {
        ROS_ERROR("Failed to get parameter, please check it");
        return 1;
    }

    nh.param("/multipointplan/enable_yaw_align", enable_yaw_align, false);
    nh.param("/multipointplan/yaw_align_thresh", yaw_align_thresh, M_PI_2);

    readpyt(yaml_path);

    odom_sub = nh.subscribe("odom_topic", 10, odom_goal_cb);
    if(flag_start_plan){
        startcommand_sub = nh.subscribe("/move_base_simple/goal", 10, startplan_cb);
    }
    px4_mode_sub = nh.subscribe("/px4ctrl/state", 10, px4_mode_cb);
    rc_sub = nh.subscribe("/mavros/rc/in", 10, rc_cb);
    takeoff_land_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
    startcommand_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    point_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 10);
    yaw_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/yaw", 10);
    timer = nh.createTimer(ros::Duration(0.01), Point_send);
    goal_modified_sub_ = nh.subscribe("/goal_modified", 10, goalModifiedCallback);

    ros::spin();
}
