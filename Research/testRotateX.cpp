#include <kernel.h>
#include "kernel_cfg.h"
#include "app.h"

#include "mbed.h"

#include "app_config.h"

#include "Zumo.h"
#include "Milkcocoa.h"

#include <bits/stdc++.h>

extern void onpush(MQTT::MessageData& md);
Serial pc(USBTX, USBRX);
Zumo zumo;

DigitalOut ledR(LED1);

DigitalIn sw1(D12);
const int buf_size = 10;
int x_offset,y_offset,z_offset;
short xFilter[buf_size], yFilter[buf_size],zFilter[buf_size];
int x,y,z;

void setOffset(){
	x_offset=0;
	y_offset=0;
	z_offset=0;
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

int getThreshold(bool clockwise, int angle, int crnt_angle){
	if(clockwise){
		if(crnt_angle + angle <= 180){
			return crnt_angle + angle;
		} else {
			return crnt_angle + angle - 360;
		}
	} else {
		if(crnt_angle - angle >= -180){
			return crnt_angle - angle;
		} else {
			return crnt_angle - angle + 360;
		}
	}
}

void task_main(intptr_t exinf) {
	int n;
	sw1.mode(PullUp);

	filtering();

	pc.printf("%d, %d ,%d \r\n", x, y, z);

	float atan = atan2(x, y);
	int deg = atan * 180 / M_PI;

	pc.printf("%f, %d \r\n", atan, deg);

	int border = getThreshold(false, 90, deg);

	pc.printf("border: %d \r\n", border);

	zumo.driveTank(-50, 50);

	int crnt_deg = deg;
	float max_atan = -100;
	float min_atan =  100;

	while(crnt_deg > border + 5 || border - 5 > crnt_deg){
		n = sw1;
		if(n == 0){
			zumo.driveTank(0, 0);
			pc.printf("end\r\n");
			break;
		}

		filtering();
		atan = atan2(x, y);
		if(atan > max_atan){
			max_atan = atan;
		}
		if(atan < min_atan){
			min_atan = atan;
		}
		crnt_deg = atan * 180 / M_PI;
		pc.printf("x,y: %d, %d atan: %f crnt_deg %d \r\n", x, y, atan, crnt_deg);
	}

	pc.printf("max,min : %f,%f \r\n", max_atan, min_atan);
	zumo.driveTank(0, 0);
	pc.printf("end \r\n");
}
