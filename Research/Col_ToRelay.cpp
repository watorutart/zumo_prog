#include <kernel.h>
#include "kernel_cfg.h"
#include "app.h"
#include "mbed.h"
#include "app_config.h"
#include "Zumo.h"
#include "Milkcocoa.h"

extern void onpush(MQTT::MessageData& md);
Serial pc(USBTX, USBRX);
Zumo zumo;

int calc_avg(int);
void resetPID();
float pid_sample(float, float, float, float, float);
float math_limit(float, float, float);
void linetrace(unsigned int[6], int, float, float, float);
void getIRvalue();

unsigned int IR_values[6];		//IRセンサ値を格納　[0]:right,[3]:center,[5]:left
int size = 10;
float diff[2] = {0, 0};
float DELTA_T = 0.001;
float integral = 0;

const static int BLACK = 250;
const static int GRAY = 150;
const static int WHITE = 100;
const static int BW_border = 250;	//B:Black, W:White
const static int GW_border = 120;	//G:Gray, W:White

float kp = 0.5;
float ki = 0.1;
float kd = 0;

DigitalOut ledR(LED1);

void onpush(MQTT::MessageData& md)
{
	pc.printf("onpush\n\r");
    MQTT::Message &message = md.message;
    DataElement de = DataElement((char*)message.payload);
	int v = de.getInt("z");
}

void task_main(intptr_t exinf) {
	int gray_check;
	int gray_count, black_count;
	int border;

	resetPID();

	do{
		//道の色を検出
		getIRvalue();
		//道の色が黒ならループを抜ける
		if(IR_values[5] >= BLACK){
			break;
		}
		linetrace(IR_values, GW_border, kp, ki, kd);
	} while(IR_values[5] >= GRAY);//道の色が灰色ならループ

	//gray_checkをリセット
	gray_check = 0;
	gray_count = 0;
	black_count = 0;

	resetPID();
	border = BW_border;

	do{
		//道の色を検出する
		getIRvalue();
		//ライントレースする
		linetrace(IR_values, border, kp, ki, kd);
		if(border == BW_border){
			//灰色の道ならgray_check++
			if(IR_values[5] >= GRAY && IR_values[5] < BLACK){
				gray_count++;
				if(gray_count >= 3){
					gray_check++;
					gray_count = 0;
					border = GW_border;
				}
			} else {
				gray_count = 0;
			}
		}
		else if(border == GW_border){
			//灰色の道ならgray_check++
			if(IR_values[5] >= BLACK){
				black_count++;
				if(gray_count >= 3){
					border = BW_border;
				}
			} else {
				black_count = 0;
			}
		}
	} while(gray_check < 2);	//gray_check>=2なら停止
	zumo.driveTank(0, 0);

	//衝突回避

	resetPID();

	do{
		//道の色を検出する
		getIRvalue();
		//ライントレースする
		linetrace(IR_values, BW_border, kp, ki, kd);
		//灰色の道ならgray_check++
		if(IR_values[5] >= GRAY && IR_values[5] < BLACK){
			gray_count++;
			if(gray_count > 3){
				break;
			}
		} else {
			gray_count = 0;
		}
	} while(1);		//灰色の道を検出したら停止


	zumo.driveTank(0, 0);
}

void getIRvalue(){
	zumo.readAnalogIrValue(IR_values);
}

int calc_avg(int value[]){
	int total = 0;

	for(int i=0;i<size;i++){
		total += value[i];
	}

	return total/size;
}

void resetPID(){
	diff[0] = 0;
	diff[1] = 0;
	integral = 0;
}

float pid_sample(float sensor_val, float target_val, float KP, float KI, float KD){
	float p, i, d;

	diff[0] = diff[1];
	diff[1] = sensor_val - target_val;
	integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;

	p = KP * diff[1];
	i = KI * integral;
	d = KD * (diff[1] - diff[0]) / DELTA_T;

	return math_limit(p + i + d, -50, 50);
}

float math_limit(float val, float min, float max){
	if(val < min){
		return min;
	} else if (val > max) {
		return max;
	}
	return val;
}

void linetrace(unsigned int IR_values[6], int border, float Kp,float Ki,float Kd){
	int left_speed = 0;
	int right_speed = 0;

	int pid_value = (int)pid_sample(IR_values[3], border, Kp,Ki,Kd);

	left_speed = 70 - pid_value;
	right_speed = 70 + pid_value;

	pc.printf("IR_value: %d, L_sp: %d, R_sp: %d \r\n", IR_values[3], left_speed, right_speed);

	zumo.driveTank(left_speed, right_speed);

	dly_tsk(100);	//Delay.msDelay(1000);
}
