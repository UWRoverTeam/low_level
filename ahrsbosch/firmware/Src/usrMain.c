/* gyro calib, gy-87
$1 = {
  x = 1.79923117,
  y = 0.531753778,
  z = 0.780779481
}

$2 = {
  x = 1.67178833,
  y = 0.185795322,
  z = -0.496351868
}
*/
#include "usrMain.h"
#include "can.h"
#include "canteros.h"
#include "globals.h"
#include "stdio.h"
#include "stdlib.h"
#include "handles.h"

int _write(int file, char *data, int len)
{
   HAL_UART_Transmit(uart1Handle, (uint8_t*)data, len, 1000);
   return 0;
}

#include "bno055_support.h"

#define PRIMARY_ADDRESS 400
#define ADDRESSES_SIZE 1
static const uint16_t myAddresses[ADDRESSES_SIZE] = {
	PRIMARY_ADDRESS
};
#define HEADERS_SIZE 1
static const uint8_t handledHeaders[HEADERS_SIZE] = {
	100 //period
};

static uint32_t sendPeriodMs = 100;

static inline void handleMessage(const CanterosMessage* m)
{
	if (m->header == 100) { //set period
		sendPeriodMs = (m->payload[0] << 8) | m->payload[1];
	}
}

int usrMain()
{
	for (int i = 1; i <= 3; ++i)
		led(i, 0);
	HAL_Delay(500);

	for (int i = 1; i <= 3; ++i) {
		led(i, 1);
		HAL_Delay(100); 
	}
	for (int i = 3; i >= 1; --i) {
		led(i, 0);
		HAL_Delay(100);
	}

	printf("test\r\n");
	uint8_t reg_data[5] = {0,0,0,0,0};
	HAL_I2C_Mem_Read(i2c1Handle, 0x28 << 1, 0, 1, reg_data, 1, 100);
	HAL_I2C_Mem_Read(i2c1Handle, 0x28 << 1, 1, 1, reg_data + 1, 1, 100);
	HAL_I2C_Mem_Read(i2c1Handle, 0x28 << 1, 2, 1, reg_data + 2, 1, 100);
	printf("ID b b b  : %d %d %d\r\n", (uint8_t)(reg_data[0]), (uint8_t)(reg_data[1]), (uint8_t)(reg_data[2]));
	reg_data[0] = 0;
	reg_data[1] = 0;
	reg_data[2] = 0;
	HAL_I2C_Mem_Read(i2c1Handle, 0x28 << 1, 0, 1, reg_data, 3, 100);
	printf("ID at once: %d %d %d\r\n", (uint8_t)(reg_data[0]), (uint8_t)(reg_data[1]), (uint8_t)(reg_data[2]));


	I2C_routine();
	bno055_init(&bno055);
	printf("libread: %d\r\n", (uint8_t)(bno055.chip_id));
	bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);

	HAL_Delay(500);

	struct bno055_accel_t accel_xyz;
	bno055_read_accel_xyz(&accel_xyz);
    printf("accel: %d, %d, %d", (int)(accel_xyz.x), (int)(accel_xyz.y), (int)(accel_xyz.z));
	
	bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF); // dodać _FMC_OFF
	struct bno055_euler_double_t d_euler_hpr;

	configCanteros(PRIMARY_ADDRESS, ADDRESSES_SIZE, myAddresses, HEADERS_SIZE, handledHeaders);
	canInit(CANMODE_CANTEROS, CANMODE_OFF);
	canterosInit();

	uint32_t lastSendMs = HAL_GetTick();
	while (1) {
		canterosProcess();
		if (canterosCheckInbox()) {
			CanterosMessage message;
			canterosGetNew(&message);
			handleMessage(&message);
		}

		uint32_t nowMs = HAL_GetTick();
		if (nowMs - lastSendMs > sendPeriodMs) {
			lastSendMs = nowMs;

			bno055_convert_double_euler_hpr_deg(&d_euler_hpr);

			CanterosMessage toSend;
			toSend.header = 106;
			toSend.payloadSize = 6;
			toSend.payload[0] = ((int16_t)(d_euler_hpr.p*10)) >> 8;
			toSend.payload[1] = ((int16_t)(d_euler_hpr.p*10)) & 0xff;
			toSend.payload[2] = ((int16_t)(d_euler_hpr.r*10)) >> 8;
			toSend.payload[3] = ((int16_t)(d_euler_hpr.r*10)) & 0xff;
			toSend.payload[4] = ((int16_t)(d_euler_hpr.h*10)) >> 8;
			toSend.payload[5] = ((int16_t)(d_euler_hpr.h*10)) & 0xff;
			HAL_StatusTypeDef res = canterosSendMessage(&toSend);
			if (res)
				led(2, 1);
		}
	}
	return 0;
}
