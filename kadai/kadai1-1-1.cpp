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

static int speed = 80;
static int steer = 0;
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

	int black = 400;
	unsigned int IR_values[6];
	zumo.driveTank(100, 100);

	while(1){
		n = sw1;
		if(n == 0){
			zumo.driveTank(0, 0);
			pc.printf("end\r\n");
			return;
		}

		zumo.readAnalogIrValue(IR_values);
		if(IR_values[0] >= black && IR_values[3] >= black && IR_values[5] >= black){
			break;
		}

		//dly_tsk(200);
	}

	pc.printf("loop end\r\n");
}
