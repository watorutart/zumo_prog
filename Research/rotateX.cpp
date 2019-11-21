#include <kernel.h>
#include "kernel_cfg.h"
#include "app.h"
#include "mbed.h"
#include "app_config.h"
#include "Zumo.h"
#include "Milkcocoa.h"
#include <stdlib.h>

void rotateX(bool, int);
int getThreshold(bool, int, int);
int calcAngle(int, int);
int calc_avg(int);
void search_avgMagne();
void setOffset(short, short, short);
void filtering();

Serial pc(USBTX, USBRX);
Zumo zumo;

DigitalOut ledR(LED1);
DigitalOut ledG(LED2);
DigitalIn sw1(D12);

const int buf_size = 10;
int x_offset,y_offset,z_offset;
short xFilter[buf_size], yFilter[buf_size],zFilter[buf_size];
int x, y, z;

void task_main(intptr_t exinf) {
	sw1.mode(PullUp);

	pc.baud(115200);

	dly_tsk(500);

	pc.printf("filtering...\r\n");
	ledR = 1;
	search_avgMagne();
	ledR = 0;
	pc.printf("filtering end \r\n");


	for(int i=0; i<4 ; i++){
		zumo.driveTank(100, 100);
		dly_tsk(300);
		zumo.driveTank(0, 0);

		rotateX(false, 90);
	}
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

