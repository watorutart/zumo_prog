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
void linetrace(bool, unsigned int[6], int, float, float, float);
void getIRvalue();
void ltEscGrayload(bool);
void ltToXGrayload(bool, int);
void ltToGrayload(bool);
void ltToXCross(bool, int);

unsigned int IR_values[6];		//IRセンサ値を格納　[0]:right,[3]:center,[5]:left
int size = 10;
float diff[2] = {0, 0};
float DELTA_T = 0.001;
float integral = 0;

const static int BLACK = 200;
const static int GRAY = 150;
const static int WHITE = 100;
const static int BW_border = 250;	//B:Black, W:White
const static int GW_border = 120;	//G:Gray, W:White
const static bool RIGHT = true;
const static bool LEFT = false;

float kp = 0.3;
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
	ltToXCross(LEFT, 2);
	ledR = 1;
	dly_tsk(1000);

	//衝突回避

	//ltToGrayload(RIGHT);

	zumo.driveTank(0, 0);
	ledR = 0;
}

void ltToXCross(bool mode, int x){
	int i = 0;
	int ir_num;
	int cross_count = 0;

	if(mode == RIGHT){
		ir_num = 0;
	} else {
		ir_num = 5;
	}

	resetPID();

	//ライントレースする
	while(i <= x) {
		//道の色を検出
		getIRvalue();

		//線の交差を検知したら
		if(IR_values[ir_num] >= BLACK){
			cross_count++;
			if(cross_count >= 2){
				i++;
				cross_count=0;
			}
		} else {
				cross_count=0;
		}
		linetrace(mode, IR_values, BW_border, kp, ki, kd);
	}

	//停止する
	zumo.driveTank(0, 0);
}


void ltEscGrayload(bool mode){
	int ir_num;

	resetPID();

	if(mode == RIGHT){
		ir_num = 5;
	} else {
		ir_num = 0;
	}

	do{
		//道の色を検出
		getIRvalue();
		//道の色が黒ならループを抜ける
		if(IR_values[ir_num] >= BLACK){
			break;
		}
		linetrace(mode, IR_values, GW_border, kp, ki, kd);
	} while(IR_values[ir_num] >= GRAY);//道の色が灰色ならループ

	zumo.driveTank(0, 0);
}

void ltToGrayload(bool mode){
	int ir_num;
	int gray_count;

	if(mode == RIGHT){
		ir_num = 5;
	} else {
		ir_num = 0;
	}

	//初期化
	gray_count = 0;
	resetPID();

	//ライントレースする
	while(1) {
		//道の色を検出する
		getIRvalue();
		//ライントレースする
		linetrace(mode, IR_values, BW_border, kp, ki, kd);
		//灰色の道ならgray_check++
		if(IR_values[ir_num] >= GRAY && IR_values[ir_num] < BLACK){
			gray_count++;
			if(gray_count > 3){
				break;
			}
		} else {
			gray_count = 0;
		}
	}

	zumo.driveTank(0, 0);
}

void ltToXGrayload(bool mode, int x){
	int i = 0;
	int gray_count;
	int ir_num;

	if(mode == RIGHT){
		ir_num = 5;
	} else {
		ir_num = 0;
	}

	//初期化
	gray_count = 0;
	resetPID();

	//ライントレースする
	while(i <= x) {
		//道の色を検出する
		getIRvalue();
		//ライントレースする
		linetrace(mode, IR_values, BW_border, kp, ki, kd);
		//灰色の道ならgray_check++
		if(IR_values[ir_num] >= GRAY && IR_values[ir_num] < BLACK){
			gray_count++;
			if(gray_count > 3){
				i++;
		        gray_count = 0;
		        ltEscGrayload(mode);
			}
		} else {
			gray_count = 0;
		}
	}

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

void linetrace(bool mode, unsigned int IR_values[6], int border, float Kp,float Ki,float Kd){
	int left_speed = 0;
	int right_speed = 0;

	int pid_value = (int)pid_sample(IR_values[3], border, Kp,Ki,Kd);

	//右端をライントレースする場合
	if(mode == RIGHT){
		left_speed = 70 + pid_value;
		right_speed = 70 - pid_value;
	}
	// 左端をライントレースする場合
	if(mode == LEFT){
		left_speed = 70 - pid_value;
		right_speed = 70 + pid_value;
	}

	pc.printf("IR_value: %d, L_sp: %d, R_sp: %d \r\n", IR_values[3], left_speed, right_speed);

	zumo.driveTank(left_speed, right_speed);

	dly_tsk(100);	//Delay.msDelay(1000);
}
