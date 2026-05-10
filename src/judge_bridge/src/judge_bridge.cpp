#include "judge_bridge/judge_bridge.hpp"
#include "judge_bridge/protocol.hpp"
#include "judge_bridge/decode.hpp"
#include "judge_bridge/serial.hpp"
/* ============================================================================
 * 文件名: judge_bridge.cpp
 * 功能说明: 
 *   - 与RoboMaster裁判系统通过串口进行通信(115200波特率)
 *   - 作为雷达和哨兵之间的通信网桥
 *   - 处理来自裁判系统的消息
 *   - 向哨兵发送雷达检测的目标信息和入侵警报
 * ============================================================================ */
#include <boost/locale.hpp>
#include <boost/locale/encoding.hpp>
#include <boost/locale/encoding_errors.hpp>
#include <boost/locale/encoding_utf.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <codecvt>
#include <cmath>
#include <locale>
#include <cstdint>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

namespace {
std::vector<std::string> get_serial_port_candidates(const std::string& configured_port)
{
    std::vector<std::string> ports;
    std::string port = configured_port;
    std::transform(port.begin(), port.end(), port.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (!configured_port.empty() && port != "auto") {
        ports.push_back(configured_port);
        return ports;
    }

    const boost::filesystem::path dev_path("/dev");
    if (!boost::filesystem::exists(dev_path))
        return ports;

    for (const auto& entry : boost::filesystem::directory_iterator(dev_path)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("ttyACM", 0) == 0 || name.rfind("ttyUSB", 0) == 0)
            ports.push_back(entry.path().string());
    }
    std::sort(ports.begin(), ports.end());
    return ports;
}
}


void JudgeBridgeNode::filter_handler(JudgeSerial::JudgePair message)
{
    switch (message.first){
        // 根据消息命令ID进行分类处理
    case CMD_ID::DETECT_PROCESS:{
        auto mark_data = reinterpret_cast<radar_mark_data_t*>(message.second.data());
        pub_radar_mark_data->publish(decode_radar_mark_data(*mark_data));
        }
        break;
    case CMD_ID::RADAR_INFO:{
        auto radar_info = reinterpret_cast<radar_info_t*>(message.second.data());
        auto radar_info_msg = decode_radar_info(*radar_info);
        pub_radar_info->publish(radar_info_msg);
        }
        break;
    case CMD_ID::RADAR_LINK_POSITION: {
        auto data = reinterpret_cast<radar_link_position_t*>(message.second.data());
        pub_radar_link_position->publish(decode_radar_link_position(*data));
        }
        break;
    case CMD_ID::RADAR_LINK_HP: {
        auto data = reinterpret_cast<radar_link_hp_t*>(message.second.data());
        pub_radar_link_hp->publish(decode_radar_link_hp(*data));
        }
        break;
    case CMD_ID::RADAR_LINK_BULLET: {
        auto data = reinterpret_cast<radar_link_bullet_t*>(message.second.data());
        pub_radar_link_bullet->publish(decode_radar_link_bullet(*data));
        }
        break;
    case CMD_ID::RADAR_LINK_COIN_AND_OCCUPY: {
        auto data = reinterpret_cast<radar_link_coin_and_occupy_t*>(message.second.data());
        pub_radar_link_coin_and_occupy->publish(decode_radar_link_coin_and_occupy(*data));
        }
        break;
    case CMD_ID::RADAR_LINK_BUFF: {
        auto data = reinterpret_cast<radar_link_buff_t*>(message.second.data());
        pub_radar_link_buff->publish(decode_radar_link_buff(*data));
        }
        break;
    case CMD_ID::RADAR_LINK_PASSWORD: {
        auto data = reinterpret_cast<radar_link_password_t*>(message.second.data());
        pub_radar_link_password->publish(decode_radar_link_password(*data));
        }
        break;
    case CMD_ID::ROBOT_STATUS:
        robot_status_callback(*reinterpret_cast<robot_status_t*>(message.second.data()));
        break;
    case CMD_ID::MAP_COMMAND:
        map_command_callback(*reinterpret_cast<map_command_t*>(message.second.data()));
        break;
    case CMD_ID::GAME_STATUS:
        game_status_callback(*reinterpret_cast<game_status_t*>(message.second.data()));
        break;
    case CMD_ID::GAME_ROBOT_HP:
        game_robot_hp_callback(*reinterpret_cast<game_robot_HP_t*>(message.second.data()));
        break;
    case CMD_ID::INTERACTION_DATA:
        interaction_data_callback(message.second);
        break;
    default:
        RCLCPP_DEBUG(rclcpp::get_logger("command"), "redundant commands");           
        return;
    }
}

void JudgeBridgeNode::send_radar_cmd(const std_msgs::msg::UInt8 &radar_cmd)
{
    if (color == team_color::UNKNOWN)
    {
        RCLCPP_WARN(get_logger(), "Unkown Color!");
        return;
    }
    // 将雷达命令转发给DV设备
    // 2026 protocol: header(cmd_id+sender_id+receiver_id) + 8-byte radar_cmd_t.
    robot_interaction_dv_data_t dv_data {};
    dv_data.header.data_cmd_id = RADAR_CMD;
    dv_data.header.sender_id = RADAR_ID[color];
    dv_data.header.receiver_id = 0x8080;
    dv_data.cmd.radar_cmd = radar_cmd.data;
    dv_data.cmd.password_cmd = static_cast<uint8_t>(
        std::clamp<int64_t>(get_parameter("radar_password_cmd").as_int(), 0, 255));

    auto password = get_parameter("radar_password").as_string();
    password.resize(6, '0');
    dv_data.cmd.password_1 = static_cast<uint8_t>(password[0]);
    dv_data.cmd.password_2 = static_cast<uint8_t>(password[1]);
    dv_data.cmd.password_3 = static_cast<uint8_t>(password[2]);
    dv_data.cmd.password_4 = static_cast<uint8_t>(password[3]);
    dv_data.cmd.password_5 = static_cast<uint8_t>(password[4]);
    dv_data.cmd.password_6 = static_cast<uint8_t>(password[5]);
    judge_serial->write(CMD_ID::INTERACTION_DATA, reinterpret_cast<uint8_t*>(&dv_data), sizeof(dv_data));
    RCLCPP_INFO(get_logger(), "DV: %d", radar_cmd.data);
}

void JudgeBridgeNode::send_custom_info(const std::string& str)
{
    // 发送自定义文本信息给飞机(aerial client)
    // 将UTF-8字符串转换为UTF-16编码(每个汉字占2字节)并通过串口发送
    if (color == team_color::UNKNOWN)
    {
        RCLCPP_WARN(get_logger(), "Unkown Color!");
        return;
    }
    custom_info_t custom_info {};
    custom_info.sender_id = RADAR_ID[color];
    custom_info.receiver_id = AERIAL_CLIENT[color];
    std::u16string u16_info = boost::locale::conv::utf_to_utf<char16_t>(str);
    if (u16_info.size() > 15)
        RCLCPP_WARN(get_logger(), "Custom info too long!");
    u16_info.resize(15);
    
    for (unsigned i = 0; i < 15; ++i)
    {
        char16_t ch = u16_info[i];
        custom_info.user_data[2 * i] = static_cast<uint8_t>(ch & 0xFF);
        custom_info.user_data[2 * i + 1] = static_cast<uint8_t>((ch >> 8) & 0xFF);
    }
    judge_serial->write(CMD_ID::SEND_CUSTOM_INFO, reinterpret_cast<uint8_t*>(&custom_info), sizeof(custom_info));
}

void JudgeBridgeNode::map_command_callback(const map_command_t& cmd)
{
    RCLCPP_INFO(get_logger(), "keyboard: %#x "
                              "x: %f, y: %f, id: %#x",
        cmd.cmd_keyboard, cmd.target_position_x, cmd.target_position_y, cmd.target_robot_id);
}

void JudgeBridgeNode::robot_status_callback(const robot_status_t& robot_data)
{
    team_color::msg color_msg;
    uint8_t radar_id = robot_data.robot_id;
/*
 * 功能: 根据机器人状态消息确定我方阵营颜色(红蓝)
 * 参数: robot_data - 包含本机器人ID的状态数据
 * 说明: 从robot_id推断阵营，并发布color消息通知其他节点
 */
    switch (radar_id){
    case RADAR_ID::R_RED: {
        if (color != team_color::C_RED)
            RCLCPP_INFO(get_logger(), "WE ARE <<<RED>>>");
        color = team_color::C_RED;
    } break;
    case RADAR_ID::R_BLUE: {
        if (color != team_color::C_BLUE)
            RCLCPP_INFO(get_logger(), "WE ARE <<<BLUE>>>");
        color = team_color::C_BLUE;
    } break;
    default:
        RCLCPP_WARN(get_logger(), "Unknow the radar id");
        return;
    }
    color_msg.data = static_cast<bool>(color);
    pub_color->publish(color_msg);
}

/**
 * 功能: 处理比赛状态消息(类型、阶段、剩余时间)
 * 参数: status - 状态结构体
 * 说明: 当比赛处于战斗阶段时发布剩余时间到话题
 */
void JudgeBridgeNode::game_status_callback(const game_status_t& status)
{
    RCLCPP_INFO(get_logger(), "game_status: game_type_and_progress: %d, remain_time: %d", status.game_type_and_progress, status.stage_remain_time);
    if ((status.game_type_and_progress >> 4) == 4)
    {
        RCLCPP_INFO(get_logger(), "game in battle");
        auto remain_time_msg = std_msgs::msg::UInt16();
        remain_time_msg.data = status.stage_remain_time;
        pub_remain_time->publish(remain_time_msg);
    }
}

/**
 * 功能: 处理机器人血量消息，转换并发布为标准ROS消息格式
 * 参数: hp - 包含红蓝队所有机器人和防御阵地的血量数据
 * 说明: 将裁判系统格式转换为GameRobotHP消息，包含:
 *       - 红队: 哨兵、英雄、工程、3号步兵、4号步兵、空中机器人
 *       - 蓝队: 哨兵、英雄、工程、3号步兵、4号步兵、空中机器人
 *       - 双方基地和前哨站血量
 */
void JudgeBridgeNode::game_robot_hp_callback(const game_robot_HP_t& hp)
{
    msg::GameRobotHP msg;
    msg.red_robot_hp = {
        hp.red_sentry_robot_HP,
        hp.red_hero_robot_HP,
        hp.red_engineer_robot_HP,
        hp.red_standard_3_robot_HP,
        hp.red_standard_4_robot_HP,
        hp.red_standard_5_robot_HP,
    };
    msg.red_base = hp.red_base_HP;
    msg.red_outpost = hp.red_outpost_HP;
    msg.blue_robot_hp = {
        hp.blue_sentry_robot_HP,
        hp.blue_hero_robot_HP,
        hp.blue_engineer_robot_HP,
        hp.blue_standard_3_robot_HP,
        hp.blue_standard_4_robot_HP,
        hp.blue_standard_5_robot_HP,
    };
    msg.blue_base = hp.blue_base_HP;
    msg.blue_outpost = hp.blue_outpost_HP;
    pub_game_robot_hp->publish(msg);
}

/**
 * 功能: 处理来自裁判系统的交互数据(地图键盘、UWB定位)
 * 参数: data - 裁判系统格式的原始二进制数据
 * 处理: 
 *   - MAP_KEYBOARD: 操作手的地图上的点击控制命令
 *   - UWB_DATA: 敌方UWB定位系统提供的位置信息
 */
void JudgeBridgeNode::interaction_data_callback(const std::vector<uint8_t>& data)
{
    auto header = reinterpret_cast<const robot_interaction_header_t*>(data.data());
    if (header->data_cmd_id == INTERACTION_CMD::MAP_KEYBOARD)
    {
        auto map_interaction = reinterpret_cast<const robot_interaction_map_data_t*>(data.data());
        radar_interface::msg::MapCommand msg;
        msg.target_position_x = map_interaction->map_cmd.target_position_x;
        msg.target_position_y = map_interaction->map_cmd.target_position_y;
        msg.target_robot_id = map_interaction->map_cmd.target_robot_id;
        msg.cmd_keyboard = map_interaction->map_cmd.cmd_keyboard;
        msg.cmd_source = map_interaction->map_cmd.cmd_source;
        pub_map_keyboard->publish(msg);
        RCLCPP_INFO(get_logger(), "Transferred Key: %d", map_interaction->map_cmd.cmd_keyboard);
    }
    else if (header->data_cmd_id == INTERACTION_CMD::UWB_DATA)
    {
        auto uwb = reinterpret_cast<const robot_interaction_uwb_t*>(data.data());
        radar_interface::msg::UwbData msg;
        msg.hero_x = uwb->hero_x;
        msg.hero_y = uwb->hero_y;
        msg.engineer_x = uwb->engineer_x;
        msg.engineer_y = uwb->engineer_y;
        msg.standard_3_x = uwb->standard_3_x;
        msg.standard_3_y = uwb->standard_3_y;
        msg.standard_4_x = uwb->standard_4_x;
        msg.standard_4_y = uwb->standard_4_y;
        msg.standard_5_x = uwb->standard_5_x;
        msg.standard_5_y = uwb->standard_5_y;
        pub_uwb_data->publish(msg);
        RCLCPP_INFO(get_logger(), "UWB Received");
    }
}

/**
 * 功能: 周期性向哨兵发送全场目标位置信息
 * 参数: topic_message - MatchResult消息(包含红蓝两队目标信息)
 * 通信: 
 *   - 把红蓝两队所有有效目标位置打包为robot_interaction_sentry_data_t结构体
 *   - 每个单位存储: robot_id, pos_x(单位m), pos_y(单位m)
 *   - 最多12个单位(对应6*2队)
 *   - 通过CMD_ID::INTERACTION_DATA命令发送给哨兵
 */
void JudgeBridgeNode::send_sentry_data(const radar_interface::msg::MatchResult& topic_message)
{
    if (color == team_color::UNKNOWN)
    {
        RCLCPP_WARN(get_logger(), "Unkown Color!");
        return;
    }
    // 构建发往哨兵的数据包
    robot_interaction_sentry_data_t interaction_data;
    interaction_data.header.data_cmd_id = INTERACTION_CMD::SENTRY_DATA;

    switch (color) {
    case team_color::C_RED:
        interaction_data.header.sender_id = RADAR_ID::R_RED;
        interaction_data.header.receiver_id = SENTRY_ID[team_color::C_RED];
        break;
    case team_color::C_BLUE:
        interaction_data.header.sender_id = RADAR_ID::R_BLUE;
        interaction_data.header.receiver_id = SENTRY_ID[team_color::C_BLUE];
        break;
    default:
        RCLCPP_WARN_ONCE(get_logger(), "Unknow the radar color");
        return;
    }
    uint8_t len = 0;

    // 遍历红队所有6个单位，依次添加有效的目标位置
    for (uint8_t index = 0; index < topic_message.red.size(); index++)
    {
        const auto& red = topic_message.red[index];
        if (red.id == -1)
            continue;
        interaction_data.custom_data[len].robot_id = RED_ROBOT[index];
        interaction_data.custom_data[len].pos_x = red.position[0];
        interaction_data.custom_data[len].pos_y = red.position[1];
        ++len;
    }

    // 遍历蓝队所有6个单位，依次添加有效的目标位置
    for (uint8_t index = 0; index < topic_message.blue.size(); index++)
    {
        const auto& blue = topic_message.blue[index];
        if (blue.id == -1)
            continue;
        interaction_data.custom_data[len].robot_id = BLUE_ROBOT[index];
        interaction_data.custom_data[len].pos_x = blue.position[0];
        interaction_data.custom_data[len].pos_y = blue.position[1];
        ++len;
    }

    interaction_data.arr_len = len;
    judge_serial->write(CMD_ID::INTERACTION_DATA, reinterpret_cast<uint8_t*>(&interaction_data),
        sizeof(robot_interaction_sentry_data_t) - (12 - len) * sizeof(robot_interaction_sentry_data_t::robot_pos));
}

/**
 * 功能: 向哨兵发送敌方入侵警报
 * 参数: msg - MatchResult消息(包含敌我双方目标)
 * 说明:
 *   - 只标记已确认进入我方半场的敌方单位
 *   - 根据雷达相对坐标系，我方防区始终在近端(x < 12)
 *   - 只发送有效目标(id != -1)且x∈(0, 12)的敌人信息
 *   - 最多发送12个入侵敌人的信息
 */
void JudgeBridgeNode::send_invasion_alert(const radar_interface::msg::MatchResult& msg)
{
    if (color == team_color::UNKNOWN)
    {
        RCLCPP_WARN(get_logger(), "Unkown Color!");
        return;
    }

    // 构建警报数据包，使用同一格式的robot_interaction_sentry_data_t
    robot_interaction_sentry_data_t interaction_data;
    interaction_data.header.data_cmd_id = INTERACTION_CMD::SENTRY_DATA;

    switch (color)
    {
    case team_color::C_RED:
        interaction_data.header.sender_id = RADAR_ID::R_RED;
        interaction_data.header.receiver_id = SENTRY_ID[team_color::C_RED];
        break;
    case team_color::C_BLUE:
        interaction_data.header.sender_id = RADAR_ID::R_BLUE;
        interaction_data.header.receiver_id = SENTRY_ID[team_color::C_BLUE];
        break;
    default:
        RCLCPP_WARN_ONCE(get_logger(), "Unknow the radar color");
        return;
    }

    // 入侵判断标准: 敌方单位相对我方的x坐标 < 12 
    constexpr double invade_x_threshold = 12.0;
    uint8_t len = 0;
    
    // 根据我方阵营分别检查敌方入侵
    if (color == team_color::C_RED)
    {
        // 我方为红队，检查蓝队敌人是否进入入侵区域(x < 12)
        for (uint8_t index = 0; index < msg.blue.size() && len < 12; ++index)
        {
            const auto& enemy = msg.blue[index];
            if (enemy.id == -1)
                continue;
            // 只记录距离我方雷达站较近的入场敌人(x < 12)
            if (enemy.position[0] <= 0.0 || enemy.position[0] >= invade_x_threshold)
                continue;
            interaction_data.custom_data[len].robot_id = BLUE_ROBOT[index];
            interaction_data.custom_data[len].pos_x = enemy.position[0];
            interaction_data.custom_data[len].pos_y = enemy.position[1];
            ++len;
        }
    }
    else
    {
        // 我方为蓝队，检查红队敌人是否进入入侵区域(x < 12)
        for (uint8_t index = 0; index < msg.red.size() && len < 12; ++index)
        {
            const auto& enemy = msg.red[index];
            if (enemy.id == -1)
                continue;
            // 因为雷达站位置随阵营转变，我方防守区域始终是相对较小的正x坐标
            if (enemy.position[0] <= 0.0 || enemy.position[0] >= invade_x_threshold)
                continue;
            interaction_data.custom_data[len].robot_id = RED_ROBOT[index];
            interaction_data.custom_data[len].pos_x = enemy.position[0];
            interaction_data.custom_data[len].pos_y = enemy.position[1];
            ++len;
        }
    }

    if (len == 0)
        return;
    // 如果没有任何敌人进入，则本次不发送警报

    interaction_data.arr_len = len;
    judge_serial->write(CMD_ID::INTERACTION_DATA, reinterpret_cast<uint8_t*>(&interaction_data),
        sizeof(robot_interaction_sentry_data_t) - (12 - len) * sizeof(robot_interaction_sentry_data_t::robot_pos));
}

    /**
     * 功能: 实时检测敌方单位是否进入我方半场，触发入侵警报
     * 参数: msg - MatchResult消息
     * 机制:
     *   - 设置300ms最小发送间隔，避免持续频发通信信息
     *   - 根据敌方位置检测：
     *     * 根据设定的相对坐标系，我方防区始终是 x < 12 的区域
     *     * 红队防卫/蓝队防卫均检查敌方单位 x ∈ (0, 12) 内的情况
     *   - 一旦检测到入侵就立即调用send_invasion_alert()发送警报
     */
void JudgeBridgeNode::check_enemy_invasion(const radar_interface::msg::MatchResult& msg)
{
    if (color == team_color::UNKNOWN) return;

    static rclcpp::Time last_alert_time(0, 0, get_clock()->get_clock_type());
    const int alert_interval_ms = 300;

    bool invaded = false;
    constexpr double invade_x_threshold = 12.0;

    if (color == team_color::C_RED) {
        // 我方为红方，蓝方进入x<12视为入侵
        for (const auto& enemy : msg.blue) {
            if (enemy.id != -1 && enemy.position[0] > 0.0 && enemy.position[0] < invade_x_threshold) {
                invaded = true;
                break;
            }
        }
    } else if (color == team_color::C_BLUE) {
        // 我方为蓝方，红方进入x<12视为入侵
        for (const auto& enemy : msg.red) {
            if (enemy.id != -1 && enemy.position[0] > 0.0 && enemy.position[0] < invade_x_threshold) {
                invaded = true;
                break;
            }
        }
    }

    if (invaded) {
        // 敌方检测到入侵：记录信息日志并向哨兵发送警报
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Enemy invaded our half map! Triggering communication alert.");
        if ((now() - last_alert_time).nanoseconds() >= static_cast<int64_t>(alert_interval_ms) * 1000000LL) {
            send_invasion_alert(msg);
            last_alert_time = now();
        }
    }
}

/**
 * 功能: 向地图和显示系统发送敌方机器人的实时位置
 * 参数: msg - MatchResult消息(包含敌方6个单位的坐标)
 * 处理流程:
 *   1. 调用check_enemy_invasion()检测敌方是否进入我方半场
 *   2. 将敌方机器人位置单位从(米)转换为地图坐标单位(厘米)
 *   3. 为离线/死亡的机器人设置默认位置(对角坐标，避免地图显示异常)
 *   4. 通过CMD_ID::ROBOT_MAP命令发送给裁判系统(供地图和UI显示)
 * 说明: 地图类型区分：
 *   - 红队：对手(蓝队)位置，默认位置为右下角(2590, 1390)
 *   - 蓝队：对手(红队)位置，默认位置为左上角(210, 110)
 */
void JudgeBridgeNode::send_map_robot_data(const radar_interface::msg::MatchResult& msg)
{
    check_enemy_invasion(msg);

    map_robot_data_t map_robot_data {};
    auto to_cm = [](double meters) -> uint16_t {
        const auto centimeters = std::lround(meters * 100.0);
        return static_cast<uint16_t>(std::clamp<long>(centimeters, 0, 65535));
    };
    auto fill_target = [&](uint16_t& x, uint16_t& y, const auto& targets, size_t index) {
        if (index >= targets.size() || targets[index].id == -1) {
            x = 0;
            y = 0;
            return;
        }
        x = to_cm(targets[index].position[0]);
        y = to_cm(targets[index].position[1]);
    };

    switch (color) {
    case team_color::C_RED: {
        fill_target(map_robot_data.opponent_hero_position_x, map_robot_data.opponent_hero_position_y, msg.blue, 1);
        fill_target(map_robot_data.opponent_engineer_position_x, map_robot_data.opponent_engineer_position_y, msg.blue, 2);
        fill_target(map_robot_data.opponent_infantry_3_position_x, map_robot_data.opponent_infantry_3_position_y, msg.blue, 3);
        fill_target(map_robot_data.opponent_infantry_4_position_x, map_robot_data.opponent_infantry_4_position_y, msg.blue, 4);
        fill_target(map_robot_data.opponent_aerial_position_x, map_robot_data.opponent_aerial_position_y, msg.blue, 5);
        fill_target(map_robot_data.opponent_sentry_position_x, map_robot_data.opponent_sentry_position_y, msg.blue, 0);
        fill_target(map_robot_data.ally_hero_position_x, map_robot_data.ally_hero_position_y, msg.red, 1);
        fill_target(map_robot_data.ally_engineer_position_x, map_robot_data.ally_engineer_position_y, msg.red, 2);
        fill_target(map_robot_data.ally_infantry_3_position_x, map_robot_data.ally_infantry_3_position_y, msg.red, 3);
        fill_target(map_robot_data.ally_infantry_4_position_x, map_robot_data.ally_infantry_4_position_y, msg.red, 4);
        fill_target(map_robot_data.ally_aerial_position_x, map_robot_data.ally_aerial_position_y, msg.red, 5);
        fill_target(map_robot_data.ally_sentry_position_x, map_robot_data.ally_sentry_position_y, msg.red, 0);
        break;
    }
    case team_color::C_BLUE: {
        fill_target(map_robot_data.opponent_hero_position_x, map_robot_data.opponent_hero_position_y, msg.red, 1);
        fill_target(map_robot_data.opponent_engineer_position_x, map_robot_data.opponent_engineer_position_y, msg.red, 2);
        fill_target(map_robot_data.opponent_infantry_3_position_x, map_robot_data.opponent_infantry_3_position_y, msg.red, 3);
        fill_target(map_robot_data.opponent_infantry_4_position_x, map_robot_data.opponent_infantry_4_position_y, msg.red, 4);
        fill_target(map_robot_data.opponent_aerial_position_x, map_robot_data.opponent_aerial_position_y, msg.red, 5);
        fill_target(map_robot_data.opponent_sentry_position_x, map_robot_data.opponent_sentry_position_y, msg.red, 0);
        fill_target(map_robot_data.ally_hero_position_x, map_robot_data.ally_hero_position_y, msg.blue, 1);
        fill_target(map_robot_data.ally_engineer_position_x, map_robot_data.ally_engineer_position_y, msg.blue, 2);
        fill_target(map_robot_data.ally_infantry_3_position_x, map_robot_data.ally_infantry_3_position_y, msg.blue, 3);
        fill_target(map_robot_data.ally_infantry_4_position_x, map_robot_data.ally_infantry_4_position_y, msg.blue, 4);
        fill_target(map_robot_data.ally_aerial_position_x, map_robot_data.ally_aerial_position_y, msg.blue, 5);
        fill_target(map_robot_data.ally_sentry_position_x, map_robot_data.ally_sentry_position_y, msg.blue, 0);
        break;
    }
    default:
        return;
    }
    // 序列化位置数据并通过串口发送给裁判系统
    judge_serial->write(CMD_ID::ROBOT_MAP, reinterpret_cast<uint8_t*>(&map_robot_data), sizeof(map_robot_data));
}

/**
 * 功能: 初始化与RoboMaster裁判系统的串口连接
 * 说明:
 *   - 获取配置参数，默认自动扫描/dev/ttyACM*和/dev/ttyUSB*
 *   - 自动重连机制：若连接失败，每1秒自动重试一次
 *   - 若ROS2已关闭则抛出异常，停止重连
 *   - enable_recorder参数可用于记录/回放串口数据(用于调试)
 */
void JudgeBridgeNode::init_serial()
{
    if (judge_serial)
        judge_serial.reset();
    while (!judge_serial) {
        // 循环重试直到成功建立连接
        std::string serial_port = get_parameter("serial_port").as_string();
        bool enable_recorder = get_parameter("enable_recorder").as_bool();
        const auto serial_ports = get_serial_port_candidates(serial_port);
        if (serial_ports.empty()) {
            RCLCPP_WARN(get_logger(), "No serial device found for judge_bridge, waiting...");
            if (!rclcpp::ok())
                throw std::runtime_error("No serial device found for judge_bridge");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        for (const auto& port : serial_ports) {
            try {
                judge_serial = std::make_unique<JudgeSerial>(port, enable_recorder);
                RCLCPP_INFO(get_logger(), "Connected to Serial %s", port.c_str());
                break;
            } catch (boost::system::system_error& e) {
                RCLCPP_WARN(get_logger(), "Connect to Serial %s failed, e.what(): %s", port.c_str(), e.what());
            }
        }
        if (!judge_serial)
            std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

/**
 * 构造函数与节点初始化流程
 * 功能说明：
 *   1. 声明ROS参数(串口号、是否记录通信):
 *      - serial_port: 串口设备文件路径，默认auto自动扫描/dev/ttyACM*和/dev/ttyUSB*
 *      - enable_recorder: 是否将串口通信数据写入日志
 *   2. 初始化串口连接(调用init_serial)
 *
 *   3. 创建发布者(pub_*)用于输出到其他ROS节点:
 *      - judge/radar_mark_data: 雷达标记进度(各单位标记进度条)
 *      - judge/radar_info: 雷达工作状态信息
 *      - judge/color: 确定的阵营颜色(红=true，蓝=false)
 *      - judge/remain_time: 比赛剩余时间(秒)
 *      - judge/game_robot_hp: 机器人血量(所有6队各单位)
 *      - judge/map_keyboard: 地图操作指令(从操作手传入)
 *      - judge/uwb_data: UWB定位数据(来自敌方系统)
 *
 *   4. 创建订阅者(sub_*)用于接收其他节点的输入:
 *      - judge/radar_cmd: 雷达命令(uint8类型)
 *      - matcher/match_result(×2): 雷达目标匹配结果
 *        * 第一个订阅：send_sentry_data回调(发送哨兵通信)
 *        * 第二个订阅：send_map_robot_data回调(发送地图位置)
 *      - judge/custom_info: 自定义文本信息(lambda函数处理)
 *
 *   5. 启动独立读取线程(read_thread):
 *      - 持续监听串口数据(阻塞式)
 *      - 解析每条收到的消息并取出cmd_id和data
 *      - 调用filter_handler()分类处理各类消息
 *      - 异常处理：连接丢失时自动调用init_serial重连
 */
JudgeBridgeNode::JudgeBridgeNode()
    : rclcpp::Node("judge_bridge")
{
    declare_parameter("serial_port", "auto");
    declare_parameter("enable_recorder", false);
    declare_parameter("radar_password_cmd", 0);
    declare_parameter("radar_password", "000000");
    
    // ============ 初始化串口 ============
    init_serial();

    // ============ 创建发布者 ============
    // 将裁判系统消息转换为ROS话题发送
    pub_radar_mark_data = create_publisher<radar_interface::msg::RadarMarkData>("judge/radar_mark_data", rclcpp::SystemDefaultsQoS());
    pub_radar_info = create_publisher<radar_interface::msg::RadarInfo>("judge/radar_info", rclcpp::SystemDefaultsQoS());
    pub_radar_link_position = create_publisher<radar_interface::msg::RadarLinkPosition>("judge/radar_link_position", rclcpp::SystemDefaultsQoS());
    pub_radar_link_hp = create_publisher<radar_interface::msg::RadarLinkHp>("judge/radar_link_hp", rclcpp::SystemDefaultsQoS());
    pub_radar_link_bullet = create_publisher<radar_interface::msg::RadarLinkBullet>("judge/radar_link_bullet", rclcpp::SystemDefaultsQoS());
    pub_radar_link_coin_and_occupy = create_publisher<radar_interface::msg::RadarLinkCoinAndOccupy>("judge/radar_link_coin_and_occupy", rclcpp::SystemDefaultsQoS());
    pub_radar_link_buff = create_publisher<radar_interface::msg::RadarLinkBuff>("judge/radar_link_buff", rclcpp::SystemDefaultsQoS());
    pub_radar_link_password = create_publisher<radar_interface::msg::RadarLinkPassword>("judge/radar_link_password", rclcpp::SystemDefaultsQoS());
    pub_color = create_publisher<radar_interface::team_color::msg>("judge/color", rclcpp::SystemDefaultsQoS());
    pub_remain_time = create_publisher<std_msgs::msg::UInt16>("judge/remain_time", rclcpp::SystemDefaultsQoS());
    pub_game_robot_hp = create_publisher<radar_interface::msg::GameRobotHP>("judge/game_robot_hp", rclcpp::SystemDefaultsQoS());
    pub_map_keyboard = create_publisher<radar_interface::msg::MapCommand>("judge/map_keyboard", rclcpp::SystemDefaultsQoS());
    pub_uwb_data = create_publisher<radar_interface::msg::UwbData>("judge/uwb_data", rclcpp::SystemDefaultsQoS());

    // ============ 创建订阅者 ============
    // 接收其他节点的消息并转发至裁判系统
    sub_radar_cmd = create_subscription<std_msgs::msg::UInt8>("judge/radar_cmd", rclcpp::SystemDefaultsQoS(), std::bind(&JudgeBridgeNode::send_radar_cmd, this, std::placeholders::_1));
    sub_match_result = create_subscription<radar_interface::msg::MatchResult>("matcher/match_result", rclcpp::SystemDefaultsQoS(), std::bind(&JudgeBridgeNode::send_sentry_data, this, std::placeholders::_1));
    sub_map_robot_data = create_subscription<radar_interface::msg::MatchResult>("matcher/match_result", rclcpp::SystemDefaultsQoS(), std::bind(&JudgeBridgeNode::send_map_robot_data, this, std::placeholders::_1));
    sub_custom_info = create_subscription<std_msgs::msg::String>("judge/custom_info", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::String& msg) {
            RCLCPP_INFO(get_logger(), "Custom Info: %s", msg.data.c_str());
            send_custom_info(msg.data);
        });

    // ============ 启动独立线程监听串口数据 ============
    read_thread = std::thread([&]() {
        while (rclcpp::ok()) {
            try {
                if (!judge_serial)
                    throw std::runtime_error("judge_serial not exists");
                auto [id, data] = judge_serial->read();
                RCLCPP_DEBUG(get_logger(), "Received msg: %#x", id);
                // 根据消息ID分类处理(详见filter_handler)
                filter_handler({ id, data });
            } catch (std::runtime_error& e) {
                // 串口异常时触发重连
                RCLCPP_WARN(get_logger(), "Error: %s", e.what());
                init_serial();
            }
        }
    });
}
