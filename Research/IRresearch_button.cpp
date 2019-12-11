#include <kernel.h>
#include "kernel_cfg.h"
#include "app.h"
#include "mbed.h"
#include "app_config.h"
#include "Zumo.h"
#include "Milkcocoa.h"

void disp_result(unsigned int[], unsigned int[], unsigned int[], int);

extern void onpush(MQTT::MessageData& md);
Serial pc(USBTX, USBRX);
Zumo zumo;

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
	int n;
	sw1.mode(PullUp);
	pc.baud(115200);

	ledR = 0;

	int size = 100;
	unsigned int L_result[size], C_result[size], R_result[size];
	unsigned int IR_values[6];
	int loop=0;

	while(loop < 100){
		zumo.readAnalogIrValue(IR_values);
		//pc.printf("%d, %d, %d \r\n", IR_values[0], IR_values[3], IR_values[5]);
		dly_tsk(100);

		L_result[loop] = IR_values[0];
		C_result[loop] = IR_values[3];
		R_result[loop] = IR_values[5];

		loop++;
	}

	pc.printf("1 loop end\r\n");
	ledR = 1;

	while(1){
		n = sw1;
		if(n == 0){
			disp_result(L_result, C_result, R_result, size);
			break;
		}
	}

	ledR = 0;

	pc.printf("2 loop end\r\n");
}

void disp_result(unsigned int L_result[], unsigned int C_result[], unsigned int R_result[], int size){
	int i;

	for(i=0;i<size;i++){
		pc.printf("%d, %d, %d \r\n", L_result[i], C_result[i], R_result[i]);
	}
}
