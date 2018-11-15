/*
  SCA150I 串口控制协议后端类     add by awesome
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_Mount_Backend.h"

#define AP_Mount_SCA150I_serial_RESEND_MS   100    // resend angle targets to gimbal once per second

class AP_Mount_SCA150I_serial : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_SCA150I_serial(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

    virtual void handle_data16(mavlink_channel_t chan, mavlink_message_t *msg); /* add by awesome  */

    virtual void send_data16(mavlink_channel_t chan); /* add by awesome  */

private:

    // read_incoming
    void read_incoming();
    bool can_send(bool with_control);
    void sca150i_pack_data(uint8_t command);
    bool control_axis(uint8_t command, const Vector3f& angle);
    void send_command(void);
    bool enter_angle(void);    /* 比较云台当前角度和指令发送的角度，3度以内不重新发送指令 */

    enum ParseStatus {
    	PARSE_STATUS_NONE = 0,
    	PARSE_STATUS_HEADER1,
    	PARSE_STATUS_HEADER2,
    	PARSE_STATUS_OK
    };

    /* 吊舱发送至飞控数据 */
    struct PACKED SCA150I_reply_ack_struct {
    	uint8_t byte1;      /* 帧头1  0xEB */
    	uint8_t byte2;      /* 帧头2  0x90 */
    	uint8_t command_page;   /* 指令页面 */
		uint8_t command;		/* 对应控制指令 */
		int16_t yaw_ang;        /* 航向角度， *0.01度 */
		int16_t pitch_ang;      /* 俯仰角度， *0.01度 */
		int16_t yaw_gyro;   	/* 航向陀螺数据， *0.01度/s */
		int16_t pitch_gyro;     /* 俯仰陀螺数据， *0.01度/s */
		uint8_t reserve2;
		uint8_t crc;
    };

    /* 飞控发送至吊舱数据 */
    struct PACKED cmd_set_gimbal_struct {
        uint8_t byte1;      /* 帧头1  0xEB */
        uint8_t byte2;      /* 帧头2  0x90 */
        uint8_t command_page;      /* 命令页面     地面端为0x00， 吊舱端为0x01 */
        uint8_t command;	    /* 命令字  */
        int16_t yaw_ang;    /* 航向角度 */
        int16_t pitch_ang;  /* 俯仰角度 */
        uint8_t yaw_drift;   /* 航向漂移补偿 */
        uint8_t pitch_drift; /* 俯仰漂移补偿 */
        uint8_t reserve1;
        uint8_t reserve2;
        uint8_t reserve3;
        uint8_t crc;
    };

    /* SCA1501I为双轴云台   */
    struct PACKED current_angle
    {
    	float pitch_angle;   /* 当前航向角   度 */
    	float yaw_angle;   /* 当前航向角   度 */
    };


    // internal variables
    AP_HAL::UARTDriver *_port;

    bool _initialised;              // true once the driver has been initialised
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal
    uint32_t _init_send;

    uint8_t _reply_counter;			/* 一帧有效数据计数 */
    enum ParseStatus _parse_status;      /* 解析数据状态 */

    bool _data16_enbale;

    union array_reply
    {
    	SCA150I_reply_ack_struct reply_data_pack;
    	uint8_t pack_data[sizeof(SCA150I_reply_ack_struct)];
    }_buffer,_buffer_gcs;

    union array_set
    {
    	cmd_set_gimbal_struct cmd_set_gimbal;
    	uint8_t pack_data[sizeof(cmd_set_gimbal_struct)];
    }_send_buffer;

    current_angle _current_angle;

    // keep the last _current_angle values
//    Vector3l _current_angle;    /* 暂时保留这个角度变量 */
};
