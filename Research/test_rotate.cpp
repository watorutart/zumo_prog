#include <kernel.h>
#include "kernel_cfg.h"
#include "app.h"

#include "mbed.h"

#include "app_config.h"

#include "Zumo.h"
#include "Milkcocoa.h"

void setOffset();
void filtering();

Serial pc(USBTX, USBRX);
Zumo zumo;

DigitalOut ledR(LED1);

const int buf_size = 10;
int x_offset,y_offset,z_offset;
short xFilter[buf_size], yFilter[buf_size],zFilter[buf_size];
int x, y, z;

void task_main(intptr_t exinf) {
	int bef_x, bef_y;
	int delta_x, delta_y;
	short temp_x, temp_y, temp_z;

	pc.baud(115200);

	pc.printf("filtering...\r\n");
	setOffset();
	filtering();
	bef_x = x;
	bef_y = y;
	pc.printf("%d %d\r\n", bef_x, bef_y);

	zumo.driveTank(100, -100);
	dly_tsk(300);

	while(1){
		zumo.getMagetism(&temp_x, &temp_y, &temp_z);
		pc.printf("%d %d\r\n", temp_x, temp_y);
		delta_x = bef_x + temp_x;
		delta_y = bef_y + temp_y;
		if(abs(delta_x) < 200 && abs(delta_y) < 200){
			zumo.driveTank(0, 0);
			break;
		}
	}

}

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
