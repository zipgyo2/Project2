#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* OSEK declarations */
DeclareCounter(SysTimerCnt);

DeclareTask(MAIN);

DeclareTask(CAR_SPEED);
DeclareTask(CAR_DIR);
DeclareTask(CAR_BRAKE);
DeclareTask(CAR_SONAR);

DeclareTask(IdleTask);

/* Gloval Values */
int speed = 0;
int str_speed = 0;
int brake_count = 0;
int sonar_array[5];
int cnt = 0, sonar_flag = 0;


/* LEJOS OSEK hooks */
void ecrobot_device_initialize()
{
	ecrobot_init_bt_connection();
	ecrobot_init_sonar_sensor(NXT_PORT_S4);
}

void ecrobot_device_terminate()
{
	ecrobot_term_bt_connection();
	ecrobot_term_sonar_sensor(NXT_PORT_S4);
}

/* LEJOS OSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
	StatusType ercd;
	ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
	if (ercd != E_OK)
	{
		ShutdownOS(ercd);
	}
}

/*CAR_SPEED Task control car-speed mode by motor */
TASK(CAR_SPEED)
{
	nxt_motor_set_speed(NXT_PORT_B, speed, 1);
	nxt_motor_set_speed(NXT_PORT_C, speed, 1);
	TerminateTask();
}

/*CAR_DIR Task control car diraction mode by motor*/
TASK(CAR_DIR)
{
	nxt_motor_set_speed(NXT_PORT_A, 0, 1);
	if (str_speed > 0)
	{
		// Angle of wheel does not exceed str_speed
		if (nxt_motor_get_count(NXT_PORT_A)<str_speed) nxt_motor_set_speed(NXT_PORT_A, 30, 1);
		else nxt_motor_set_speed(NXT_PORT_A, 0, 1);
	}
	else if (str_speed < 0)
	{
		// Angle of wheel does not exceed str_speed
		if (nxt_motor_get_count(NXT_PORT_A)>str_speed) nxt_motor_set_speed(NXT_PORT_A, -30, 1);
		else nxt_motor_set_speed(NXT_PORT_A, 0, 1);
	}
	else if (str_speed == 0) {
		nxt_motor_set_speed(NXT_PORT_A, 0, 1);
	}

	TerminateTask();
}

/* CAR_BRAKE Task control brake mode(Immediately, or Slowly) */
TASK(CAR_BRAKE)
{
	//Immediate brake mode
	if (speed == 0) {
		nxt_motor_set_speed(NXT_PORT_A, 0, 0);
		nxt_motor_set_speed(NXT_PORT_B, 0, 0);
		nxt_motor_set_speed(NXT_PORT_C, 0, 0);

	}
	//Slow brake mode
	else
	{
		nxt_motor_set_speed(NXT_PORT_B, speed, 1);
		nxt_motor_set_speed(NXT_PORT_C, speed, 1);
	}

	TerminateTask();
}

/* CAR_SONAR Task control Sonar mode */
TASK(CAR_SONAR)
{
	if (sonar_flag == 1) {
		int temp;
		int sonar;
		sonar = ecrobot_get_sonar_sensor(NXT_PORT_S4);
		if (sonar<70) {
			sonar_array[cnt++] = sonar;
		}
		if (cnt >= 5) {
			for (int i = 0; i<5; i++) {
				for (int j = i + 1; j<5; j++) {
					if (sonar_array[i]>sonar_array[j]) {
						temp = sonar_array[i];
						sonar_array[i] = sonar_array[j];
						sonar_array[j] = temp;
					}
				}
			}
			sonar = sonar_array[2];
			cnt = 0;
			if (sonar >= 50 && sonar<70)
				ecrobot_sound_tone(208, 400, 40);
			else if (sonar >= 30 && sonar<50)
				ecrobot_sound_tone(262, 400, 60);
			else if (sonar >= 10 && sonar<30)
				ecrobot_sound_tone(392, 400, 80);
			else if (sonar<10) {
				ecrobot_sound_tone(523, 400, 100);
				speed = 0;
				ActivateTask(CAR_BRAKE);
			}
		}
	}
	TerminateTask();
}
/* EventDispatcher executed every 5ms */
TASK(MAIN)
{
	//Static Value
	static U8 bt_receive_buf[32];
	static U8 TouchSensorStatus_old = 0;

	//Value
	int new_speed = 0;

	/* read packet data from the master device */
	ecrobot_read_bt_packet(bt_receive_buf, 32);
	if (bt_receive_buf[7] == 2) {
		//Sonar mode on
		sonar_flag = 1;
	}
	else if (bt_receive_buf[7] != 2) {
		//Sonar mode off
		sonar_flag = 0;
	}
	if (bt_receive_buf[7] == 1) {
		if (bt_receive_buf[6] == 2) {
			//Slow brake mode
			if (brake_count == 100 || brake_count == 0)
			{
				brake_count = 0;
				if (brake_count == 100)brake_count = 0;
				new_speed = speed + (-1)*speed / 5;
				if (new_speed*speed<0) speed = 0;
				if (new_speed*speed>0) speed = new_speed;
				ActivateTask(CAR_BRAKE);
			}
			brake_count++;
		}
		else if (bt_receive_buf[6] == 1)
		{
			//Immediate brake mode
			speed = 0;
			ActivateTask(CAR_BRAKE);
		}

	}
	else if (bt_receive_buf[7] != 2) {
		sonar_flag = 0;
		if (bt_receive_buf[5] == 1)
		{
			//Fast speed mode
			if (bt_receive_buf[3] == 1)
			{
				//Forward
				speed = -100;
				ActivateTask(CAR_SPEED);
			}
			else if (bt_receive_buf[3] == 2)
			{
				//Backward
				speed = 100;
				ActivateTask(CAR_SPEED);
			}
		}
		else if (bt_receive_buf[5] == 2) 
		{
			//Slow speed mode
			if (bt_receive_buf[3] == 1)
			{
				//Forward
				speed = -30;
				ActivateTask(CAR_SPEED);
			}
			else if (bt_receive_buf[3] == 2)
			{
				//Backward
				speed = 30;
				ActivateTask(CAR_SPEED);
			}
		}
		else {
			//Default speed mode
			if (bt_receive_buf[3] == 1)
			{
				//Forward
				speed = -50;
				ActivateTask(CAR_SPEED);

			}
			else if (bt_receive_buf[3] == 2)
			{
				//Backward
				speed = 50;
				ActivateTask(CAR_SPEED);
			}
		}
		// car direction mode
		if (bt_receive_buf[4] == 3)
		{
			//Right
			str_speed = -90;
			ActivateTask(CAR_DIR);
		}
		else if (bt_receive_buf[4] == 4)
		{
			//Left
			str_speed = 90;
			ActivateTask(CAR_DIR);
		}
		else
		{
			str_speed = 0;
			ActivateTask(CAR_DIR);
		}
	}
	TouchSensorStatus = ecrobot_get_touch_sensor(NXT_PORT_S4);
	if (TouchSensorStatus == 1 && TouchSensorStatus_old == 0)
	{
		/* Send a Touch Sensor ON Event to the Handler */
		SetEvent(EventHandler, TouchSensorOnEvent);
	}
	else if (TouchSensorStatus == 0 && TouchSensorStatus_old == 1)
	{
		/* Send a Touch Sensor OFF Event to the Handler */
		SetEvent(EventHandler, TouchSensorOffEvent);
	}
	TouchSensorStatus_old = TouchSensorStatus;

	TerminateTask();
}

/* IdleTask */
TASK(IdleTask)
{
	static SINT bt_status = BT_NO_INIT;

	while (1)
	{
		ecrobot_init_bt_slave("NXT");

		if (ecrobot_get_bt_status() == BT_STREAM && bt_status != BT_STREAM)
		{
			display_clear(0);
			display_goto_xy(0, 0);
			display_string("[BT]");
			display_update();
		}
		bt_status = ecrobot_get_bt_status();
	}
}
