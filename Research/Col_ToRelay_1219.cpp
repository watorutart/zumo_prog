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
void leftLinetrace(bool, unsigned int[6], int, float, float, float);
void getIRvalue();
void ltEscGrayload(bool);
void ltToXGrayload(bool, int);
void ltToGrayload(bool);
void ltToXCross(bool, int);
void ltToCross(bool);
void leftLtToCross(bool);
void ltToWhiteload(bool);
void leftLtEscCross(bool);
void ltEscCross(bool);

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

const static int BLACK = 330;
const static int GRAY = 220;
const static int WHITE = 100;
const static int BW_border = 250;	//B:Black, W:White
const static int GW_border = 150;	//G:Gray, W:White
const static bool RIGHT = true;
const static bool LEFT = false;

const int buf_size = 10;
int x_offset,y_offset,z_offset;
short xFilter[buf_size], yFilter[buf_size],zFilter[buf_size];
int x, y, z;

int base_speed = 60;

float kp = 0.3;
float ki = 0.1;
float kd = 0.00005;

int test = 0;

DigitalOut ledR(LED1);
DigitalOut ledG(LED2);
DigitalOut ledB(LED3);

DigitalIn sw1(D12);

void onpush(MQTT::MessageData& md)
{
	pc.printf("onpush\n\r");
    MQTT::Message &message = md.message;
    DataElement de = DataElement((char*)message.payload);
	int v = de.getInt("z");
}

void task_main(intptr_t exinf) {
	int n;
	while(1){
		n=sw1;
		if(n == 0){
			break;
		}
	}

	//地磁気センサのキャリブレーション
	search_avgMagne();

	ltEscGrayload(RIGHT);  //これは無い方が安定するかも?

	ltToWhiteload(RIGHT);

	ledR = 1;
	dly_tsk(1000);

	rotateX(false, 30);

	ltToCross(LEFT);

	ledR = 0;
	ledG = 1;
	dly_tsk(1000);

	//衝突回避

	ltEscCross(LEFT);
	//ltEscGrayload(LEFT);
	//ledG = 1;

	rotateX(true, 40);

	kp = 0.4;
	leftLtToCross(LEFT);

	ledR = 1;
	dly_tsk(1000);

	leftLtEscCross(LEFT);
	ledR=0;
	ledG = 0;

	rotateX(true, 10);

	kd *= 2;

	test = 70;

	ltToGrayload(RIGHT);
	ledG=1;
}

void ltEscCross(bool mode){
	int ir_num = 5;

	//resetPID();

	do{
		//道の色を検出
		getIRvalue();
		//道の色が黒ならループを抜ける
		if(IR_values[ir_num] <= WHITE){
			break;
		}
		linetrace(mode, IR_values, BW_border, kp, ki, kd);
	} while(IR_values[ir_num] >= BLACK);//道の色が灰色ならループ

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
		linetrace(mode, IR_values, GW_border, kp, ki, kd);
	} while(IR_values[ir_num] >= BLACK);//道の色が灰色ならループ

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
		if(IR_values[ir_num] >= BLACK){
			base_speed /= 2;
			cross_check++;
			if(cross_check >= 3){
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


void ltToCross(bool mode){
	int ir_num;
	int cross_check;

	if(mode == RIGHT){
		ir_num = 0;
	} else {
		ir_num = 5;
	}

	//初期化
	cross_check = 0;
	resetPID();

	//ライントレースする
	while(1) {
		//道の色を検出
		getIRvalue();

		//線の交差を検知したら
		if(IR_values[ir_num] >= BLACK){
			base_speed /= 2;
			cross_check++;
			if(cross_check >= 3){
				break;
			}
		} else {
			cross_check = 0;
			base_speed = 70;
		}
		linetrace(mode, IR_values, BW_border, kp, ki, kd);
	}

	base_speed = 70;
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
	} while(IR_values[ir_num] >= GRAY && IR_values[ir_num] < BLACK);//道の色が灰色ならループ

	zumo.driveTank(0, 0);
}

void ltToWhiteload(bool mode){
	int ir_num;
	int white_count;

	if(mode == RIGHT){
		ir_num = 5;
	} else {
		ir_num = 0;
	}

	//初期化
	white_count = 0;
	resetPID();

	//ライントレースする
	while(1) {
		//道の色を検出する
		getIRvalue();
		//ライントレースする
		linetrace(mode, IR_values, BW_border, kp, ki, kd);
		//灰色の道ならgray_check++
		if(IR_values[ir_num] < WHITE){
			white_count++;
			if(white_count >= 5){
				break;
			}
		} else {
			white_count = 0;
		}
	}

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
			base_speed /= 2;
			gray_count++;
			if(gray_count >= 3){
				break;
			}
		} else {
			gray_count = 0;
			base_speed = 70;
		}
	}

	base_speed = 70;
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
	pc.printf("%d, %d, %d \r\n", IR_values[0], IR_values[3], IR_values[5]);
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

	return math_limit(p + i + d, -150, 50);
}

float math_limit(float val, float min, float max){
	if(val < min){
		return min;
	} else if (val > max) {
		return max;
	}
	return val;
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

void linetrace(bool mode, unsigned int IR_values[6], int border, float Kp,float Ki,float Kd){
	int left_speed = 0;
	int right_speed = 0;

	int pid_value = (int)pid_sample(IR_values[3], border, Kp,Ki,Kd);

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
