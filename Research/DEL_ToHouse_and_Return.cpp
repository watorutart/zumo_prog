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
void rightLinetrace(bool, unsigned int[6], int, float, float, float);
void getIRvalue();
void ltToCross();
void ltToGrayload();
void ltToXCross(int);
void rightLtToCross(bool);
void rightLtEscCross(bool);
void leftLtEscCross(bool);
void leftLtToCross(bool);
void leftLinetrace(bool, unsigned int[6], int, float, float, float);

void rotateX(bool, int);
int getThreshold(bool, int, int);
int calcAngle(int, int);
int calc_avg(int);
void search_avgMagne();
void setOffset(short, short, short);
void filtering();

unsigned int IR_values[6];		//IRセンサ値を格納　[0]:right,[3]:center,[5]:left
int size = 10;
float diff[2] = {0, 0};
float DELTA_T = 0.001;
float integral = 0;

const static int BLACK = 300;
const static int GRAY = 150;
const static int WHITE = 100;
const static int BW_border = 250;	//B:Black, W:White
const static int GW_border = 120;	//G:Gray, W:White

const static bool RIGHT = true;
const static bool LEFT = false;

const int buf_size = 10;
int x_offset,y_offset,z_offset;
short xFilter[buf_size], yFilter[buf_size],zFilter[buf_size];
int x, y, z;

const static int addr_x = 1;		//address = 0 ~ 3
const static int addr_y = 2;		//address = 0 ~ 3

float kp = 0.3;
float ki = 0.1;
float kd = 0;

int base_speed = 70;

DigitalOut ledR(LED1);
DigitalIn sw1(D12);

void onpush(MQTT::MessageData& md)
{
	pc.printf("onpush\n\r");
    MQTT::Message &message = md.message;
    DataElement de = DataElement((char*)message.payload);
	int v = de.getInt("z");
}

void task_main(intptr_t exinf) {
	//地磁気センサのキャリブレーション
	search_avgMagne();

	//現在地
	int pos_x = 0;
	int pos_y = 0;

	for(;pos_x<addr_x;pos_x++){
		rightLtEscCross(RIGHT);
		rightLtToCross(RIGHT);
	}
	ledR = 1;

	//機体を反時計回りに90度回転する
	rotateX(false, 50);

	for(;pos_y<addr_y;pos_y++){
		rightLtEscCross(RIGHT);
		rightLtToCross(RIGHT);	//受取人宅へ到着
	}
	ledR = 0;

	dly_tsk(500);

	//以下帰り道
	rotateX(true, 170);

	for(;pos_y > 0;pos_y--){
		leftLtEscCross(LEFT);
		leftLtToCross(LEFT);
	}
	ledR = 1;

	rotateX(true, 70);

	for(;pos_x > 0;pos_x--){
		leftLtEscCross(LEFT);
		leftLtToCross(LEFT);
	}
	ledR = 0;

	//機体を時計回りに45度回転
	//rotateX(45);
}

void ltToXCross(int x){
	int i = 0;
	int cross_count = 0;

	resetPID();

	//ライントレースする
	while(i <= x) {
		//道の色を検出
		getIRvalue();

		//線の交差を検知したら
		if(IR_values[5] >= BLACK){
			cross_count++;
			if(cross_count >= 3){
				i++;
				cross_count=0;
			}
			else {
				cross_count=0;
			}
		}
		linetrace(IR_values, BW_border, kp, ki, kd);
	}

	//停止する
	zumo.driveTank(0, 0);
}

void ltToCross(){
	int cross_check;
	Timer t;

	//初期化
	cross_check = 0;
	resetPID();
	t.reset();
	t.start();

	//ライントレースする
	while(1) {
		//道の色を検出
		getIRvalue();

		//線の交差を検知したら
		if(t > 1500 && IR_values[0] >= BLACK){
			cross_check++;
			if(cross_check >= 3){
				break;
			}
		} else {
			cross_check = 0;
		}
		linetrace(IR_values, BW_border, kp, ki, kd);
	}

	//停止する
	zumo.driveTank(0, 0);
}

void rightLtToCross(bool mode){
	int ir_num = 5;
	int cross_check;

	//初期化
	cross_check = 0;
	resetPID();

	//ライントレースする
	while(1) {
		//道の色を検出
		getIRvalue();

		//線の交差を検知したら
		if(IR_values[ir_num] >= GRAY){
			base_speed /= 2;
			cross_check++;
			if(cross_check >= 3){
				break;
			}
		} else {
			cross_check = 0;
			base_speed = 70;
		}
		rightLinetrace(mode, IR_values, BW_border, kp, ki, kd);
	}

	base_speed = 70;
	//停止する
	zumo.driveTank(0, 0);
}

void rightLtEscCross(bool mode){
	int ir_num = 5;

	resetPID();

	do{
		//道の色を検出
		getIRvalue();
		//道の色が白ならループを抜ける
		if(IR_values[ir_num] <= WHITE){
			break;
		}
		rightLinetrace(mode, IR_values, GW_border, kp, ki, kd);
	} while(IR_values[ir_num] >= GRAY);//道の色が灰色ならループ

	zumo.driveTank(0, 0);
}

void leftLtEscCross(bool mode){
	int ir_num = 0;

	resetPID();

	do{
		//道の色を検出
		getIRvalue();
		//道の色が黒ならループを抜ける
		if(IR_values[ir_num] <= WHITE){
			break;
		}
		leftLinetrace(mode, IR_values, GW_border, kp, ki, kd);
	} while(IR_values[ir_num] >= GRAY);//道の色が灰色ならループ

	zumo.driveTank(0, 0);
}

void leftLtToCross(bool mode){
	int ir_num = 0;
	int cross_check;

	//初期化
	cross_check = 0;
	resetPID();

	//ライントレースする
	while(1) {
		//道の色を検出
		getIRvalue();

		//線の交差を検知したら
		if(IR_values[ir_num] >= GRAY){
			base_speed /= 2;
			cross_check++;
			if(cross_check >= 2){
				break;
			}
		} else {
			cross_check = 0;
			base_speed = 70;
		}
		leftLinetrace(mode, IR_values, BW_border, kp, ki, kd);
	}

	base_speed = 70;
	//停止する
	zumo.driveTank(0, 0);
}

void ltToGrayload(){
	int gray_count;

	//初期化
	gray_count = 0;
	resetPID();

	//ライントレースする
	while(1) {
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

void rightLinetrace(bool mode, unsigned int IR_values[6], int border, float Kp,float Ki,float Kd){
	int left_speed = 0;
	int right_speed = 0;

	int pid_value = (int)pid_sample(IR_values[0], border, Kp,Ki,Kd);

	//右端をライントレースする場合
	if(mode == RIGHT){
		left_speed = base_speed + pid_value;
		right_speed = base_speed - pid_value;
	}
	// 左端をライントレースする場合
	if(mode == LEFT){
		left_speed = base_speed - pid_value;
		right_speed = base_speed + pid_value;
	}

	//pc.printf("IR_value: %d, L_sp: %d, R_sp: %d \r\n", IR_values[3], left_speed, right_speed);

	zumo.driveTank(left_speed, right_speed);

	dly_tsk(100);	//Delay.msDelay(1000);
}

void leftLinetrace(bool mode, unsigned int IR_values[6], int border, float Kp,float Ki,float Kd){
	int left_speed = 0;
	int right_speed = 0;

	int pid_value = (int)pid_sample(IR_values[5], border, Kp,Ki,Kd);

	//右端をライントレースする場合
	if(mode == RIGHT){
		left_speed = base_speed + pid_value;
		right_speed = base_speed - pid_value;
	}
	// 左端をライントレースする場合
	if(mode == LEFT){
		left_speed = base_speed - pid_value;
		right_speed = base_speed + pid_value;
	}

	//pc.printf("IR_value: %d, L_sp: %d, R_sp: %d \r\n", IR_values[3], left_speed, right_speed);

	zumo.driveTank(left_speed, right_speed);

	dly_tsk(100);	//Delay.msDelay(1000);
}

void rotateX(bool clockwise, int x_angle){
	int n = -1;
	filtering();
	dly_tsk(300);

	int deg = calcAngle(x, y);
	int border;
	border = getThreshold(clockwise, x_angle, deg);

	if(clockwise){
		zumo.driveTank(70, -70);
	} else {
		zumo.driveTank(-70, 70);
	}
	while(1){
		n = sw1;
		if(n == 0){
			zumo.driveTank(0, 0);
			pc.printf("end\r\n");
			return;
		}

		filtering();
		pc.printf("%d %d\r\n", x, y);

		if(abs(border - calcAngle(x, y)) < 5){
			zumo.driveTank(0, 0);
			break;
		}
	}
}

int getThreshold(bool clockwise, int angle, int crnt_angle){
	if(clockwise){
		if(crnt_angle + angle <= 180){
			return crnt_angle + angle;
		} else {
			return crnt_angle + angle -360;
		}
	} else {
		if(crnt_angle - angle >= -180){
			return crnt_angle - angle;
		} else {
			return crnt_angle - angle + 360;
		}
	}
}

int calcAngle(int tx, int ty){
	return atan2(y, x) * 180 / M_PI;
}

int calc_avg(int value[], int size){
	int total = 0;

	for(int i=0;i<size;i++){
		total += value[i];
	}

	return total/size;
}

void search_avgMagne(){
	int x_ip, y_ip;
	int temp_x[500];
	int temp_y[500];
	int loop = 0;

	short cx, cy, cz;

	setOffset(0, 0, 0);
	filtering();
	x_ip = x;
	y_ip = y;

	zumo.driveTank(70, -70);

	while(1){
		zumo.getMagetism(&cx, &cy, &cz);

		pc.printf("search: %d %d \r\n", cx, cy);

		if(loop > 50 && abs(x_ip-cx) < 80 && abs(y_ip-cy) < 80){
			zumo.driveTank(0, 0);
			break;
		}

		temp_x[loop++] = cx;
		temp_y[loop] = cy;

		dly_tsk(10);
	}
	int avg_x = calc_avg(temp_x, loop);
	int avg_y = calc_avg(temp_y, loop);

	pc.printf("avg x, avg y = %d, %d \r\n", avg_x, avg_y);
	setOffset(avg_x, avg_y, 0);
	dly_tsk(100);
}

void setOffset(short x, short y, short z){
	x_offset=x;
	y_offset=y;
	z_offset=z;
}

void filtering(){
	short cnt,ring;
	cnt=ring=0;
	int i;
	while(cnt <= 20){
		zumo.getMagetism(&xFilter[ring], &yFilter[ring], &zFilter[ring]);
		cnt++;
		ring=cnt%buf_size;
		x=y=z=0;
		for (i=0;i<buf_size;i++){
			x+=xFilter[i];
			y+=yFilter[i];
			z+=zFilter[i];
		}
		x/=buf_size;
		y/=buf_size;
		z/=buf_size;
		x-=x_offset;
		y-=y_offset;
		z-=z_offset;
	}
}
