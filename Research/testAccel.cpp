#include <kernel.h>
#include "kernel_cfg.h"
#include "app.h"

#include "mbed.h"

#include "app_config.h"

#include "Zumo.h"
#include "Milkcocoa.h"

extern void onpush(MQTT::MessageData& md);
void setOffset();
void filtering();
short avg(short*, int);
void disp_result(short*, short*, int);

Serial pc(USBTX, USBRX);
Zumo zumo;

DigitalOut ledR(LED1);

DigitalIn sw1(D12);

const int buf_size = 10;
int x_offset,y_offset,z_offset;
short xFilter[buf_size], yFilter[buf_size],zFilter[buf_size];
int x, y, z;

void onpush(MQTT::MessageData& md)
{
	pc.printf("onpush\n\r");
    MQTT::Message &message = md.message;
    DataElement de = DataElement((char*)message.payload);
	int v = de.getInt("z");
}

void task_main(intptr_t exinf) {
	int n;
	sw1.mode(PullUp);
	pc.baud(115200);

	short X_data[100];
	short Y_data[100];

	int loop = 0;

	setOffset();

	zumo.driveTank(100, 100);

	while(loop < 100){
		n = sw1;
		if(n == 0){
			pc.printf("1 push\r\n");
			zumo.driveTank(0, 0);
			break;
		}

		filtering();
		pc.printf("%d, %d, %d \r\n", x, y, z);
		dly_tsk(10);

		X_data[loop] = x;
		Y_data[loop] = y;

		loop++;
	}

	zumo.driveTank(0, 0);
	pc.printf("loop end\r\n");

	while(1) {
		n = sw1;
		if(n == 0){
			pc.printf("1 push\r\n");
			disp_result(X_data, Y_data, 100);
			break;
		}
	}


}

void setOffset(){
	x_offset=y_offset=z_offset=0;
	for (int i=0;i<buf_size*20;i++){
		zumo.getAcceleration(&xFilter[0], &yFilter[0], &zFilter[0]);
		x_offset+=xFilter[0];
		y_offset+=yFilter[0];
		z_offset+=zFilter[0];
		dly_tsk(50);
	}
	x_offset/=(buf_size*20);
	y_offset/=(buf_size*20);
	z_offset/=(buf_size*20);
}

void disp_result(short X_result[], short Y_result[], int size){
	int i;

	for(i=0;i<size;i++){
		pc.printf("%d,%d\r\n", X_result[i], Y_result[i]);
	}
}

void filtering(){
	short cnt,ring;
	cnt=ring=0;
	int i;
	while(cnt <= 20){
		zumo.getAcceleration(&xFilter[ring], &yFilter[ring], &zFilter[ring]);
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
