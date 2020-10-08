#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* OSEK declarations */
DeclareCounter(SysTimerCnt);

DeclareTask(MAIN);

DeclareTask(CAR_SPEED);
DeclareTask(CAR_DIR);
DeclareTask(CAR_BRAKE);

DeclareTask(IdleTask);

/* Gloval Variables */
int speed = 0;
int str_speed = 0;
int brake_count = 0;


/* LEJOS OSEK hooks */
void ecrobot_device_initialize()
{
	ecrobot_init_bt_connection();
}

void ecrobot_device_terminate()
{
	ecrobot_term_bt_connection();
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
	//set motor to speed
	nxt_motor_set_speed(NXT_PORT_B, speed, 1);
	nxt_motor_set_speed(NXT_PORT_C, speed, 1);
	TerminateTask();
}

/*CAR_DIR Task control car diraction mode by motor*/
TASK(CAR_DIR)
{
	//Initialize motor speed for immediate change of speed
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

/* EventDispatcher executed every 5ms */
TASK(MAIN)
{
	//Static Variable
	static U8 bt_receive_buf[32];

	//Variable
	int new_speed = 0;

	/* read packet data from the master device */
	ecrobot_read_bt_packet(bt_receive_buf, 32);
	if (bt_receive_buf[7] == 1) {
		if (bt_receive_buf[6] == 2) {//Slow brake mode
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
		else if (bt_receive_buf[6] == 1)//Immediate brake mode
		{
			speed = 0;
			ActivateTask(CAR_BRAKE);
		}

	}
	else if (bt_receive_buf[7] != 2) {
		if (bt_receive_buf[5] == 1)//Fast speed mode
		{
			if (bt_receive_buf[3] == 1)//Forward
			{
				speed = -100;
				ActivateTask(CAR_SPEED);
			}
			else if (bt_receive_buf[3] == 2)//Backward
			{
				speed = 100;
				ActivateTask(CAR_SPEED);
			}
		}
		else if (bt_receive_buf[5] == 2) //Slow speed mode
		{
			if (bt_receive_buf[3] == 1)//Forward
			{
				speed = -30;
				ActivateTask(CAR_SPEED);
			}
			else if (bt_receive_buf[3] == 2)//Backward
			{
				speed = 30;
				ActivateTask(CAR_SPEED);
			}
		}
		else {//Default speed mode
			if (bt_receive_buf[3] == 1)//Forward
			{
				speed = -50;
				ActivateTask(CAR_SPEED);

			}
			else if (bt_receive_buf[3] == 2)//Backward
			{
				speed = 50;
				ActivateTask(CAR_SPEED);
			}
		}
		// car direction mode
		if (bt_receive_buf[4] == 3)//Right
		{
			str_speed = -90;
			ActivateTask(CAR_DIR);
		}
		else if (bt_receive_buf[4] == 4)//Left
		{
			str_speed = 90;
			ActivateTask(CAR_DIR);
		}
		else
		{
			str_speed = 0;
			ActivateTask(CAR_DIR);
		}
	}

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
