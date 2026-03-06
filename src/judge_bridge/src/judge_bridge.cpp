#include "judge_bridge/judge_bridge.hpp"
#include "judge_bridge/protocol.hpp"
#include "judge_bridge/decode.hpp"
#include "judge_bridge/serial.hpp"
/* ============================================================================
 * 文件名: judge_bridge.cpp
 * 功能说明: 
 *   - 与RoboMaster裁判系统通过串口进行通信(115200波特率)
 *   - 作为雷达和哨兵之间的通信网桥
 *   - 处理来自裁判系统的消息(机器人状态、游戏状态、交互数据等)
 *   - 向哨兵发送雷达检测的目标信息和入侵警报
 * ============================================================================ */
#include <boost/locale.hpp>
#include <boost/locale/encoding.hpp>
#include <boost/locale/encoding_errors.hpp>
#include <boost/locale/encoding_utf.hpp>
#include <codecvt>
#include <locale>
#include <cstdint>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>


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
    // 将雷达命令转发给DV设备
    // 数据结构: header(cmd_id+sender_id+receiver_id) + radar_cmd
    robot_interaction_dv_data_t dv_data;
    dv_data.header.data_cmd_id = RADAR_CMD;
    dv_data.header.sender_id = RADAR_ID[color];
    dv_data.header.receiver_id = 0x8080;
    dv_data.radar_cmd = radar_cmd.data;
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
 *       - 红队: 哨兵、英雄、工程、3/4/5步兵各部队血量
 *       - 蓝队: 哨兵、英雄、工程、3/4/5步兵各部队血量
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

    map_robot_data_t map_robot_data;
    constexpr uint16_t default_red_x = 210, default_red_y = 110;
    constexpr uint16_t default_blue_x = 2800 - 210, default_blue_y = 1500 - 110;
    switch (color) {
    case team_color::C_RED:
        // 我方为红队，应发送蓝队敌人位置；若敌人离线则使用默认位置
        map_robot_data.sentry_position_x = msg.blue[0].id != -1 ? msg.blue[0].position[0] * 100 : default_blue_x;
        map_robot_data.sentry_position_y = msg.blue[0].id != -1 ? msg.blue[0].position[1] * 100 : default_blue_y;
        map_robot_data.hero_position_x = msg.blue[1].id != -1 ? msg.blue[1].position[0] * 100 : default_blue_x;
        map_robot_data.hero_position_y = msg.blue[1].id != -1 ? msg.blue[1].position[1] * 100 : default_blue_y;
        map_robot_data.engineer_position_x = msg.blue[2].id != -1 ? msg.blue[2].position[0] * 100 : default_blue_x;
        map_robot_data.engineer_position_y = msg.blue[2].id != -1 ? msg.blue[2].position[1] * 100 : default_blue_y;
        map_robot_data.infantry_3_position_x = msg.blue[3].id != -1 ? msg.blue[3].position[0] * 100 : default_blue_x;
        map_robot_data.infantry_3_position_y = msg.blue[3].id != -1 ? msg.blue[3].position[1] * 100 : default_blue_y;
        map_robot_data.infantry_4_position_x = msg.blue[4].id != -1 ? msg.blue[4].position[0] * 100 : default_blue_x;
        map_robot_data.infantry_4_position_y = msg.blue[4].id != -1 ? msg.blue[4].position[1] * 100 : default_blue_y;
        map_robot_data.infantry_5_position_x = msg.blue[5].id != -1 ? msg.blue[5].position[0] * 100 : default_blue_x;
        map_robot_data.infantry_5_position_y = msg.blue[5].id != -1 ? msg.blue[5].position[1] * 100 : default_blue_y;
        break;
    case team_color::C_BLUE:
        // 我方为蓝队，应发送红队敌人位置；若敌人离线则使用默认位置
        map_robot_data.sentry_position_x = msg.red[0].id != -1 ? msg.red[0].position[0] * 100 : default_red_x;
        map_robot_data.sentry_position_y = msg.red[0].id != -1 ? msg.red[0].position[1] * 100 : default_red_y;
        map_robot_data.hero_position_x = msg.red[1].id != -1 ? msg.red[1].position[0] * 100 : default_red_x;
        map_robot_data.hero_position_y = msg.red[1].id != -1 ? msg.red[1].position[1] * 100 : default_red_y;
        map_robot_data.engineer_position_x = msg.red[2].id != -1 ? msg.red[2].position[0] * 100 : default_red_x;
        map_robot_data.engineer_position_y = msg.red[2].id != -1 ? msg.red[2].position[1] * 100 : default_red_y;
        map_robot_data.infantry_3_position_x = msg.red[3].id != -1 ? msg.red[3].position[0] * 100 : default_red_x;
        map_robot_data.infantry_3_position_y = msg.red[3].id != -1 ? msg.red[3].position[1] * 100 : default_red_y;
        map_robot_data.infantry_4_position_x = msg.red[4].id != -1 ? msg.red[4].position[0] * 100 : default_red_x;
        map_robot_data.infantry_4_position_y = msg.red[4].id != -1 ? msg.red[4].position[1] * 100 : default_red_y;
        map_robot_data.infantry_5_position_x = msg.red[5].id != -1 ? msg.red[5].position[0] * 100 : default_red_x;
        map_robot_data.infantry_5_position_y = msg.red[5].id != -1 ? msg.red[5].position[1] * 100 : default_red_y;
        break;
    default:
        return;
    }
    // 序列化位置数据并通过串口发送给裁判系统
    judge_serial->write(CMD_ID::ROBOT_MAP, reinterpret_cast<uint8_t*>(&map_robot_data), sizeof(map_robot_data));
}

/**
 * 功能: 初始化与RoboMaster裁判系统的串口连接
 * 说明:
 *   - 获取配置参数(端口号通常为/dev/ttyUSB0)
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
        try {
            judge_serial = std::make_unique<JudgeSerial>(serial_port, enable_recorder);
        } catch (boost::system::system_error& e) {
            RCLCPP_WARN(get_logger(), "Connect to Serial %s failed, e.what(): %s", serial_port.c_str(), e.what());
            if (!rclcpp::ok())
                throw e;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

/**
 * 构造函数与节点初始化流程
 * 功能说明：
 *   1. 声明ROS参数(串口号、是否记录通信):
 *      - serial_port: 串口设备文件路径(默认/dev/ttyUSB0)
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
    declare_parameter("serial_port", "/dev/ttyUSB0");
    declare_parameter("enable_recorder", false);
    
    // ============ 初始化串口 ============
    init_serial();

    // ============ 创建发布者 ============
    // 将裁判系统消息转换为ROS话题发送
    pub_radar_mark_data = create_publisher<radar_interface::msg::RadarMarkData>("judge/radar_mark_data", rclcpp::SystemDefaultsQoS());
    pub_radar_info = create_publisher<radar_interface::msg::RadarInfo>("judge/radar_info", rclcpp::SystemDefaultsQoS());
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
