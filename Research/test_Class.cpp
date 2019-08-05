#include <kernel.h>
#include "kernel_cfg.h"
#include "app.h"

#include "mbed.h"

#include "app_config.h"

#include "Zumo.h"
#include "Milkcocoa.h"

Serial pc(USBTX, USBRX);
Zumo zumo;

static int speed = 80;
static int steer = 0;
DigitalOut ledR(LED1);

class Test{
public:
	int number;
	char name[4];
	void ShowData();
};

void Test::ShowData(){
	//cout << number << "\r\n";
	//cout << name << "\r\n";
	pc.printf("test.number: %d \r\n", number);
	pc.printf("test.name: %s \r\n", name);
	return;
}

void task_main(intptr_t exinf) {
	Test test;
	pc.baud(115200);

	test.number = 1;
	strcpy(test.name, "aa");

	dly_tsk(100);

	//pc.printf("\r\n class member: %d, %s \r\n", test.number, test.name);
	test.ShowData();
}
