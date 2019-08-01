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
	Timer timer1;
	int t;

	short X_data[100], Y_data[100];
	short X_result[100], Y_result[100];

	int loop = 0;
	int l_count = 0;

	sw1.mode(PullUp);
	pc.baud(115200);

	setOffset();

	//zumo.driveTank(100, 100);

	for(int i=0;i<100;i++){
		X_data[i] = 0;
		Y_data[i] = 0;
	}

	while(1){
		n = sw1;
		if(n == 0){
			pc.printf("1 push\r\n");

			zumo.driveTank(0, 0);

			while(n != 1){
				pc.printf("n = 0\r\n");
				n = sw1;
			}

			pc.printf("push end\r\n");

			while(true){
				n = sw1;
				if(n == 0){
					pc.printf("2 push\r\n");

					//add line
					disp_result(X_result, Y_result, l_count);

					pc.printf("end\r\n");
					return;
				}
			}
		}

		if(loop >= 100){
			X_result[l_count] = avg(X_data, 100);
		    Y_result[l_count] = avg(Y_data, 100);
		    l_count++;
			zumo.driveTank(100, -100);
			dly_tsk(100);
			zumo.driveTank(0, 0);
			dly_tsk(50);
			loop = 0;
		}

		filtering();
		//pc.printf("%d, %d, %d \r\n", x, y, z);

		X_data[loop] = x;
		Y_data[loop] = y;

		loop++;
		dly_tsk(10);
	}

	pc.printf("loop end\r\n");
}

short avg(short data[], int size){
	int i;
	long temp = 0;
	short result = 0;

	//pc.printf("data: %d\r\n", data[37]);

	for(i=0;i<size;i++){
		//pc.printf("data: %d\r\n", data[i]);
		temp += data[i];
		//pc.printf("temp: %d\r\n", temp);
	}

	result = temp/size;

	//pc.printf("%d\r\n", result);

	return result;
}

void disp_result(short X_result[], short Y_result[], int size){
	int i;

	for(i=0;i<size;i++){
		pc.printf("%d,%d\r\n", X_result[i], Y_result[i]);
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
