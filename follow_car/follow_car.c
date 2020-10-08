#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "math.h" //for atan2()
#include "stdlib.h" //for abs()

/* define macro for NXT PORT */
#define A NXT_PORT_A
#define B NXT_PORT_B
#define C NXT_PORT_C
#define S1 NXT_PORT_S1
#define S2 NXT_PORT_S2
#define S3 NXT_PORT_S3
#define S4 NXT_PORT_S4

#define RAD 57.2958 //Radian

/* Declare Counter, Event, Task */
DeclareCounter(SysTimerCnt);

DeclareEvent(MovementEvent);
DeclareEvent(SteeringEvent);

DeclareTask(Steering);
DeclareTask(Movement);
DeclareTask(Sonar);

/* Ecrobot Device Initialize */
void ecrobot_device_initialize()
{
    ecrobot_init_sonar_sensor(S3);
    ecrobot_init_sonar_sensor(S4);
}

void ecrobot_device_terminate()
{
    ecrobot_term_sonar_sensor(S3);
    ecrobot_term_sonar_sensor(S4);
}

/* LEJOS OSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
    StatusType ercd;

    ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
    if (ercd != E_OK)
        ShutdownOS(ercd);
}

/* Global Variable */
int left_sonar_sensor;
int right_sonar_sensor;

/* Auto-run Task -------------------------------------*/
TASK(Sonar) //15ms마다 입력 받음
{
    static int left_sonar_array[5] = {15, 15, 15, 15, 15}; //좌측 초음파 센서 배열 안전거리인 15로 배열 초기화
    static int right_sonar_array[5] = {15, 15, 15, 15, 15}; //우측 초음파 센서 배열 안전거리인 15로 배열 초기화

    static U8 entry_count = 0; //Circular Queue를 구현하기 위한 변수

    int i, j = 0;
    int left_sort[5], right_sort[5];
    int temp;

    /*sliding window 형식으로 sonar_array에 새로운값 받음*/
    left_sonar_array[entry_count] = ecrobot_get_sonar_sensor(S3);
    right_sonar_array[entry_count] = ecrobot_get_sonar_sensor(S4);

    /* Max calibration : 초음파 센서의 값이 100이상일 시 100으로 고정 */
    if (left_sonar_array[entry_count] >= 100)
    {
        left_sonar_array[entry_count] = 100;
    }
    if (right_sonar_array[entry_count] >= 100)
    {
        right_sonar_array[entry_count] = 100;
    }

    /* 초음파 센서에서 받은 값을 정렬을 위한 배열에 복사 */
    for (i = 0; i < 5; i++)
    {
        left_sort[i] = left_sonar_array[i];
        right_sort[i] = right_sonar_array[i];
    }

    /* Median calibration 초음파 센서 배열을 정렬 */
    for (i = 0; i < 5; i++)
    {
        for (j = 0; j < 5 - i; j++)
        {
            if (left_sort[j] > left_sort[j + 1])
            {
                temp = left_sort[j];
                left_sort[j] = left_sort[j + 1];
                left_sort[j + 1] = temp;
            }
            if (right_sort[j] > right_sort[j + 1])
            {
                temp = right_sort[j];
                right_sort[j] = right_sort[j + 1];
                right_sort[j + 1] = temp;
            }
        }
    }

    /* Result of calibrations : 정렬된 값에서 중간 값인 배열의 중앙 값을 초음파 측정 값으로 설정. 
        Task 실행 시 마다 센서 값 결정 */
    left_sonar_sensor = left_sort[2];
    right_sonar_sensor = right_sort[2];

    /* Steering Event Set. 좌/우 초음파 센서의 값의 차이가 있을 시 
        즉 angle 이 존재할 경우 */
    if (left_sonar_sensor - right_sonar_sensor != 0)
    {
        SetEvent(Steering, SteeringEvent);
    }

    /* Movement Event Set. 이전에 측정한 값과 현재 측정된 값이 변화 있을 시 */
    int avg = (left_sonar_array[entry_count] + right_sonar_array[entry_count]) / 2;
    int prev_avg = 0;
    if (entry_count == 0)
    {
        prev_avg = (left_sonar_array[5] + right_sonar_array[5]) / 2;
    }
    else
    {
        prev_avg = (left_sonar_array[entry_count - 1] + right_sonar_array[entry_count - 1]) / 2;
    }
    if (avg - prev_avg != 0)
    {
        // distance에 변화가 있을경우 event set
        SetEvent(Movement, MovementEvent);
    }

    /* Entry Count Rotate. Circular 방식으로 순환하게끔 배열 초기화 */
    entry_count++;
    if (entry_count == 5)
    {
        entry_count = 0;
    }

    TerminateTask();
}

TASK(Steering)
{
    while (WaitEvent(SteeringEvent) == E_OK)
    {
        ClearEvent(SteeringEvent);
        /*Static Variable*/
        static int steering_count = 0;
        static double previous_steer = 0.0;
        const double sonar_width = 15.5;

        //두 초음파 센서의 값의 차이를 통해 앞차의 회전 정도를 각도로 계산
        double d = (double)left_sonar_sensor - (double)right_sonar_sensor;
        double angle = atan2(d, sonar_width);
        angle = angle * RAD;

        /*측정된 각도가 노이즈 값이 들어오면 이전에 측정된 값으로 사용
            튀는값 보정*/
        if (angle >= 80 || angle <= -80)
        {
            angle = previous_steer;
        }

        //차량 회전 최대 각도를 벗어나면 최대 각도를 45도로 고정
        if (d > 0 && angle > 45.0)
        {
            angle = 45.0;
        }
        else if (d < 0 && (angle < -45.0 || angle > 45.0)) //angle 값이 양수로만 튀기때문에 d와같이 사용하여 보정
        {
            angle = -45.0;
        }
        /*
        angle의 각도 범위 -45 ~ 45
        steer motor count범위 -90~90
        따라서 angle에 1.8혹은 2.0을 곱하여 보정
        */ 
        //angle의 값이 25보다 크면 차량 회전 최대 값을 보정해주고, 모터의 속도를 조금 더 빠르게 설정
        steering_count = nxt_motor_get_count(B);
        if (angle > 25)
        {
            if (steering_count < angle * 2.0)
            {
                nxt_motor_set_speed(B, 30, 1);
            }
            else if (steering_count > angle * 2.0)
            {
                //steer가 왼쪽으로 잘 안돌아가기 때문에 왼쪽에 speed를 크게준다
                nxt_motor_set_speed(B, -32, 1);
            }
            //앞 바퀴의 모터가 최대각까지 진행하였다면 모터를 멈춘다.
            else
            {
                nxt_motor_set_speed(B, 0, 1);
            }
        }
        //angle 값이 25보다 작으면 차량 회전 최대 값을 angle의 1.8배 만큼 보정 해주고 모터의 속도는 default 값으로 설정
        else
        {
            if (steering_count < angle * 1.8)
            {
                nxt_motor_set_speed(B, 28, 1);
            }
            else if (steering_count > angle * 1.8)
            {
                //steer가 왼쪽으로 잘 안돌아가기 때문에 왼쪽에 speed를 크게준다
                nxt_motor_set_speed(B, -30, 1);
            }
            //앞 바퀴의 모터가 최대각까지 진행하였다면 모터를 멈춘다.
            else
            {
                nxt_motor_set_speed(B, 0, 1);
            }
        }

        previous_steer = angle;
    }
}
TASK(Movement)
{
    while (1)
    {
        WaitEvent(MovementEvent);
        ClearEvent(MovementEvent);
        static int speed = 0;
        int left_sensor = left_sonar_sensor;
        int right_sensor = right_sonar_sensor;

        int avg = (left_sensor + right_sensor) / 2; //두 초음파 센서의 평균 거리
        int diff = (avg - 15); //안전거리를 기준으로 두 차량의 거리 차이
        const int low = 47; //저속 모드의 모터의 speed 값
        const int high = 92; //고속 모드의 모터의 speed 값

        /* 앞차와의 거리가 80 이상일 시 차량 정지. 
            앞의 차를 놓쳤을 경우 사고 방지를 위함 */
        if(avg >= 80){
            nxt_motor_set_speed(A, 0, 1);
            nxt_motor_set_speed(C, 0, 1);
        }
        else
        {
            if (diff > 18) //앞차와의 간격이 커질 경우 고속 모드의 스피드로 설정
            {
                speed = high;
                if (diff > 25) //앞차와의 간격이 더 커질 경우 고속 모드의 스피드에서 조금 더 추가
                {
                    speed = speed + 5;
                }
            }
            else if (diff < -2) //앞차와의 간격이 좁아지면 후진
            {
                speed = -low - 5;
            }
            else if (diff > 2) //앞차와의 간격이 생기면 저속 모드로 전진
            {
                speed = low;
                if(diff > 10) //앞차와의 간격이 벌어지기 시작하면 저속 모드보다 조금 더 빠르게
                {
                    speed = speed + 5;
                }
            }
            // 차량이 안전거리 +-2 사이에 있으면 차량을 멈춘다.
            if (diff < 2 && diff > -2)
            {
                speed = 0;
            }

            //속도를 재설정하기 위해 초기화 한 다음 speed 값을 지정
            nxt_motor_set_speed(A, 0, 1);
            nxt_motor_set_speed(C, 0, 1);
            nxt_motor_set_speed(A, -speed, 1);
            nxt_motor_set_speed(C, -speed, 1);
        }
    }

    TerminateTask();
}