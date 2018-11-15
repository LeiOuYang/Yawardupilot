/*
  �����ǿ�˫�����   ��Ժ�Э���ӿ���   SCA150-Iϵ��   add by awesome
 */
#include "AP_Mount_SCA150I_serial.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_HAL/utility/RingBuffer.h>

/*-------- add by awesome --------*/
#include <GCS_MAVLink/GCS.h>
/*-------- end add --------*/

extern const AP_HAL::HAL& hal;

AP_Mount_SCA150I_serial::AP_Mount_SCA150I_serial(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _port(nullptr),
    _initialised(false),
    _last_send(0),
    _init_send(0),
    _reply_counter(0),
    _parse_status(PARSE_STATUS_NONE),
    _data16_enbale(false)   /* �ڷɿ��������õ���ǰ�����͵���վ��������ָ��  */
{}

/* ��ʼ����̨  */
void AP_Mount_SCA150I_serial::init(const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_SCA150I, 0);   /* ʶ���Ѿ����úõĴ��� */
    if (_port) {

    	uint8_t i = 0;
        _initialised = true;
        set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());

        for(i=0; i<sizeof(SCA150I_reply_ack_struct); ++i)
        {
        	_buffer_gcs.pack_data[i] = 0x00;
        	_buffer.pack_data[i] = 0x00;
        }
        for(i=0; i<sizeof(cmd_set_gimbal_struct); ++i)
		{
			_send_buffer.pack_data[i] = 0x00;
			_send_buffer.pack_data[i] = 0x00;
		}

        sca150i_pack_data(0x01);   /* �ϵ��ʼ��ָ�� ��������  */
        _current_angle = {0.0, 0.0};
    }
}

/* ѭ��������̨����״̬ */
void AP_Mount_SCA150I_serial::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    read_incoming(); /* �ɿؽ����ӵ��ջ�ȡ������ �� ����100ms������ͨ��DATA16����������վ */

    switch(get_mode())
    {
		case MAV_MOUNT_MODE_RETRACT:
			break;

		/* һ��ģʽ���������ģʽ */
		case MAV_MOUNT_MODE_NEUTRAL:
			if(!_data16_enbale)   /* ֻ��data16�����ݷ�����ŷ���ָ�� */
			{
				send_command();
				_data16_enbale = true;
			}
			break;

		/* ����������̨ģʽ */
		case MAV_MOUNT_MODE_MAVLINK_TARGETING:
		{
			control_axis(0x07, _angle_ef_target_rad);
			send_command();
		}
			break;

		/* RC���� */
		case MAV_MOUNT_MODE_RC_TARGETING:
			// update targets using pilot's rc inputs
			update_targets_from_rc();  /* ��ң�ػ�ȡ�Ƕȿ��ƣ� ����  */
			control_axis(0x07, _angle_ef_target_rad);
			send_command();
			//resend_now = true;
			break;

		/* ��ʱ�����κδ��� */
		case MAV_MOUNT_MODE_GPS_POINT:
			break;

		default:
			// we do not know this mode so do nothing
			break;
    }

    if ((AP_HAL::millis() - _last_send) > AP_Mount_SCA150I_serial_RESEND_MS*80) {    /* �����ڵ���û������������ɿ�  ����Ϊ������ɿ�����ʧЧ  */
    	/* ��ʾ�������Ӵ��� */
    	gcs().send_text(MAV_SEVERITY_INFO, "SCA150I gimbal comm error");
    	_last_send = AP_HAL::millis();
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_SCA150I_serial::has_pan_control() const
{
    // we do not have yaw control
    return false;
}

// set_mode - sets mount's mode
void AP_Mount_SCA150I_serial::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

/* �´�����ԭʼ���� */
void AP_Mount_SCA150I_serial::send_data16(mavlink_channel_t chan)
{
	mavlink_msg_data16_send(chan, 0x01, 14, (const uint8_t *)(_buffer_gcs.pack_data));
}

/* �´���������    MSGΪDATA16 */
void AP_Mount_SCA150I_serial::status_msg(mavlink_channel_t chan)
{
    // return target angles as gimbal's actual attitude.
    mavlink_msg_mount_status_send(chan, 0, 0, _current_angle.pitch_angle, 0.0, _current_angle.yaw_angle);

}

bool AP_Mount_SCA150I_serial::can_send(bool with_control) {
    uint16_t required_tx = 1;
    if (with_control) {
        required_tx += sizeof(mavlink_data16_t);
    }
    return (_port->txspace() >= required_tx);
}

/* ��ȡ���ڻ��������ݣ����ҽ���  */
void AP_Mount_SCA150I_serial::read_incoming() {

	uint8_t data;
    int16_t numc;

    numc = _port->available();   /* �ӵ��ն�ȡ���� */

    if (numc < 0 ){
        return;
    }

    for (int16_t j = 0; j < numc; j++) {        /* ����������� */

        data = _port->read();

        /* ���մ��͸��ɿص����ݸ���Ϊ 15λ  */
		switch(_parse_status)
		{
			case PARSE_STATUS_NONE:
				_reply_counter = 0;
				if(0xEB==data)
				{
					_parse_status = PARSE_STATUS_HEADER1;
					_buffer.pack_data[_reply_counter++] = data;
				}

				break;

			case PARSE_STATUS_HEADER1:
				if(0x90==data)
				{
					_parse_status = PARSE_STATUS_HEADER2;
					_buffer.pack_data[_reply_counter++] = data;
				}
				else
					_parse_status = PARSE_STATUS_NONE;
				break;

			case PARSE_STATUS_HEADER2:
				 _buffer.pack_data[_reply_counter++] = data;
				 if(14==_reply_counter)
				 {
					 uint32_t crc = 0;
					 uint8_t i = 0;

					 for(i=0; i<14; ++i)
					 {
						 if(0==i || 1==i)
							 continue;
						 if(13==i && static_cast<uint8_t>(crc%256)==_buffer.pack_data[13])
						 {
							 _parse_status = PARSE_STATUS_OK;
							if(PARSE_STATUS_OK==_parse_status)
							{
								/* �������õ���mavlink���ݷ���������վ  */
								_buffer_gcs.reply_data_pack = _buffer.reply_data_pack;

								/* z����Ƕȣ� y�����Ƕ�  */
								_current_angle.yaw_angle =  0.01*(static_cast<int16_t>(_buffer.pack_data[5] + (_buffer.pack_data[4]<<8)));
								_current_angle.pitch_angle =  0.01*(static_cast<int16_t>(_buffer.pack_data[7] + (_buffer.pack_data[6]<<8)));
								//gcs().send_text(MAV_SEVERITY_INFO, "yaw=%5.1f, pitch=%5.1f",_current_angle.yaw_angle, _current_angle.pitch_angle);
								_parse_status = PARSE_STATUS_NONE;
								_last_send = AP_HAL::millis();
							}else
							{
								_parse_status = PARSE_STATUS_NONE;
							}
							 break;
						 }else if(13==i)
						 {
							 _parse_status = PARSE_STATUS_NONE;
							 break;
						 }
						 crc += _buffer.pack_data[i];
					 }
				 }
				 break;
			default : break;
		}
    }
}

/* ����վ���������̨���ƽ���  add by awesome  */
void AP_Mount_SCA150I_serial::handle_data16(mavlink_channel_t chan, mavlink_message_t *msg) {

	mavlink_data16_t data16;
	mavlink_msg_data16_decode(msg, &data16);
	//mavlink_msg_data16_send_struct(chan, &data16);

	if(MAV_MOUNT_MODE_NEUTRAL!=get_mode()) return;

	_data16_enbale = true;    /* ���յ�����˵�data16���ݣ� ��ʹ�ܵ���˿��Ƶ��� */

	if(0x01==data16.type)  /* 0x01Ϊ��Э����  */
	{
		//gcs().send_text(MAV_SEVERITY_INFO, "SCA150I data16");
		uint8_t i = 0;
		if(can_send(true) && _data16_enbale)
		{
			for(i=0; i<data16.len; ++i)
			{
				_send_buffer.pack_data[i] = data16.data[i];
				_data16_enbale = false;
				_last_send = AP_HAL::millis();
			}
		}
	}
}

/* �ɿط��������տ������ݴ��  */
void AP_Mount_SCA150I_serial::sca150i_pack_data(uint8_t command)
{
	uint8_t i = 0;
	uint32_t crc = 0;

	_send_buffer.cmd_set_gimbal.byte1 = 0xEB;
	_send_buffer.cmd_set_gimbal.byte2 = 0x90;
	_send_buffer.cmd_set_gimbal.command_page = 0x00;
	_send_buffer.cmd_set_gimbal.command = command;
	_send_buffer.cmd_set_gimbal.yaw_ang = 0;
	_send_buffer.cmd_set_gimbal.pitch_ang = 0;
	_send_buffer.cmd_set_gimbal.yaw_drift = 0x7F;
	_send_buffer.cmd_set_gimbal.pitch_drift = 0x7F;
	_send_buffer.cmd_set_gimbal.reserve1 = 0x00;
	_send_buffer.cmd_set_gimbal.reserve2 = 0xFF;
	_send_buffer.cmd_set_gimbal.reserve3 = 0xFF;

	for( i=0; i<13; ++i )
	{
		if(0==i || 1==i) continue;
		crc += _send_buffer.pack_data[i];
	}
	_send_buffer.pack_data[13] = static_cast<uint8_t>(crc%256);
}

/* ����ģʽ����̨�Ƕȿ��� */
bool AP_Mount_SCA150I_serial::control_axis(uint8_t command, const Vector3f& angle)
{
	uint8_t i = 0;
	uint32_t crc = 0;
	int16_t temp = 0;
    Vector3f target_deg = angle;

    if(get_mode()==MAV_MOUNT_MODE_RC_TARGETING)   /* ����ת���ɽǶ� */
    {
    	target_deg.y = degrees(target_deg.y);
    	target_deg.z = degrees(target_deg.z);
    }

    _send_buffer.pack_data[0] = 0xEB;
    _send_buffer.pack_data[1] = 0x90;
    _send_buffer.pack_data[2] = 0x00;
    _send_buffer.pack_data[3] = command;

    temp = static_cast<int16_t>(target_deg.z*100);
	_send_buffer.pack_data[4] = static_cast<uint8_t>(((static_cast<uint16_t>(temp))>>8)&0x0ff);
    _send_buffer.pack_data[5] = static_cast<uint8_t>(((static_cast<uint16_t>(temp))&0x0ff));

    temp = static_cast<int16_t>(target_deg.y*100);
    _send_buffer.pack_data[6] = static_cast<uint8_t>(((static_cast<uint16_t>(temp))>>8)&0x0ff);
    _send_buffer.pack_data[7] = static_cast<uint8_t>(((static_cast<uint16_t>(temp))&0x0ff));

	_send_buffer.pack_data[8] = 0x7F;
	_send_buffer.pack_data[9] = 0x7F;
	_send_buffer.pack_data[10] = 0x00;
	_send_buffer.pack_data[11] = 0xFF;
	_send_buffer.pack_data[12] = 0xFF;

	for( i=0; i<13; ++i )
	{
		if(0==i || 1==i) continue;
		crc += _send_buffer.pack_data[i];
	}
	_send_buffer.pack_data[13] = static_cast<uint8_t>(crc%256);
	return true;

}

void AP_Mount_SCA150I_serial::send_command(void)
{
	uint8_t i = 0;
	if(can_send(true))
	{
		for(i=0; i<14; ++i)
		{
			/* ���������ݷ���������  */
			_port->write(_send_buffer.pack_data[i]);
		}
	}
}

/* ��̨��ǰ�ǶȺ�ָ��ǶȽ��бȽϣ����2�Ȳ����� */
bool AP_Mount_SCA150I_serial::enter_angle(void)
{
	if( fabs( fabs(static_cast<int16_t>(_angle_ef_target_rad.z*100))-fabs(static_cast<int16_t>(_current_angle.yaw_angle*100)) )<=200.000000\
	    &&  fabs( fabs(static_cast<int16_t>(_angle_ef_target_rad.y*100))-fabs(static_cast<int16_t>(_current_angle.pitch_angle*100)) )<=200.000000 )
	{
		return true;
	}else
		return false;
}


