#include "usrMain.h"
#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "handles.h"
#include "math.h"

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define abs(a) (((a)>0)?(a):-(a))


#define DEVICE_ID 211

#define UART_BUFFER_SIZE 250
volatile uint8_t uart_buffer[UART_BUFFER_SIZE] = "";
volatile uint8_t uart_message[UART_BUFFER_SIZE] = "";
volatile uint8_t uart_tx_buffer[UART_BUFFER_SIZE] = "";
volatile uint16_t uart_tx_buffer_end = 0, uart_tx_buffer_begin = 0;

uint32_t uart_count = 0;
uint32_t uart_last_count = 0;
int32_t last_position = 0;
uint8_t uart_message_len = 0;
uint32_t position_info_period_ms, position_info_next_send;
uint32_t seconds = 0;
float step_size = 3600.0f/40000;

uint32_t watchdog = 500;
uint32_t watchdog_last_time = -1000000;


enum Mode {STOPPED, POWER, POSITION, VELOCITY};
volatile enum Mode mode;
volatile float velocity, power;
volatile uint8_t rotational_encoder = 1;

#define MAX_POWER 32768L
#define MAX_PWM 3999L
volatile float target_power, last_power, current_power;
volatile int32_t target_position = 0;
volatile float standard_velocity = 5200.0f, real_velocity, target_velocity, last_target_velocity = 0, acceleration_time = 0.5f, compensation_factor = 0;
volatile float velocity_decimal_remainder = 0;
volatile uint8_t pid_enabled = 1;
volatile int32_t min_power = MAX_POWER / 20;

volatile float enc_a_level = 0.5, enc_b_level = 0.5, enc_i_level = 0.5;
const float enc_remainder = 0.9, enc_new = 0.1, enc_high_threshold = 0.65, enc_low_threshold = 0.35;
volatile uint8_t enc_a, enc_b, enc_i, enc_a_last, enc_b_last;

volatile int32_t encoder_value, index_pulse_position;

int32_t debug_last_pos = 0;


volatile uint32_t count_control, time1, time2, iter, db1, db2;

#define CONTROL_LOOP_FREQ 100
const float CONTROL_LOOP_PERIOD = 1.0f / CONTROL_LOOP_FREQ; // to avoid slow float division later


void encoder_tick()
{
    enc_a_level *= enc_remainder;
    enc_a_level += enc_new * HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin);
    if (enc_b_level < enc_low_threshold)
        enc_b = 0;
    if (enc_b_level > enc_high_threshold)
        enc_b = 1;

    enc_b_level *= enc_remainder;
    enc_b_level += enc_new * HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin);
    if (enc_a_level < enc_low_threshold)
        enc_a = 0;
    if (enc_a_level > enc_high_threshold)
        enc_a = 1;

    if(enc_a != enc_a_last)
    {
        if (enc_a != enc_b)
            ++encoder_value;
        else
            --encoder_value;
    }
    if(enc_b != enc_b_last)
    {
        if (enc_a != enc_b)
            --encoder_value;
        else
            ++encoder_value;
    }
    enc_a_last = enc_a;
    enc_b_last = enc_b;
    
    enc_i_level *= enc_remainder;
    enc_i_level += enc_new * HAL_GPIO_ReadPin(ENC_I_GPIO_Port, ENC_I_Pin);
    if (enc_i_level < enc_low_threshold)
        enc_i = 0;
    if (enc_i_level > enc_high_threshold)
        enc_i = 1;

    if (enc_i)
       index_pulse_position = encoder_value;
    db2 = encoder_value;
}

void control_tick()
{

    int32_t enc = encoder_value;
    if (rotational_encoder && enc * step_size > 3600)
    {    
        enc -= 3600 / step_size;
        encoder_value = enc;
    }
    if (rotational_encoder && enc < 0)
    {
        enc += 3600 / step_size;
        encoder_value = enc;
    }
    int32_t ind = index_pulse_position;
    if (rotational_encoder && ind * step_size > 3600)
    {
        ind -= 3600 / step_size;
        index_pulse_position = ind;
    }
    if (rotational_encoder && ind < 0)
    {
        ind += 3600 / step_size;
        index_pulse_position = ind;
    }
    if (rotational_encoder && target_position * step_size > 3600)
    {
        target_position -= 3600 / step_size;
    }
    if (rotational_encoder && target_position < 0)
    {
        target_position += 3600 / step_size;
    }


    count_control++;
    if (count_control % 100 == 0)
    {   
        HAL_GPIO_TogglePin(LED_Y_GPIO_Port, LED_Y_Pin);
        // count = sprintf((char*)buffer, "t1-t2: %ld uc: %ld encoder: %ld pos: %ld lastpow: %ld, A: %ld, B: %ld\r\n", (int32_t)(time2-time1), 
            // (int32_t)uart_count, (int32_t)encoder_value, (int32_t)target_position, (int32_t)(last_power * 1000), (int32_t)(enc_a_level*1000), (int32_t)(enc_b_level*1000));
        // uart_transmit(buffer, count);
        // debug_last_pos = encoder_value;
        //HAL_StatusTypeDef status;
        //status = 
        can_transmit(14, 0, 0, seconds>>16, seconds>>8, seconds, 0, 0);
        //status = can_transmit(11,22,33,44,55,66,77,88);
        ++seconds;
        //count = sprintf((char*)buffer, "status: %ld if ok would be: %ld \r\n", (int32_t)status, (int32_t)HAL_OK);
        //uart_transmit(buffer, count);
        //    can_transmit(200,210,220,230,240,250,190,180);
    }
    if (count_control % (position_info_period_ms / 10) == 0)
    {   
        
        uint8_t sign_pos = 0, sign_index = 0;    
        uint32_t posToSend = abs(enc) * step_size * 128;
        uint32_t indexToSend = abs(ind) * step_size * 128;
        if (enc < 0)
        {
            sign_pos = 0b10000000;
        }
        if (ind < 0)
        {
            sign_index = 0b10000000;
        }

        can_transmit(28, (uint8_t)(posToSend >> 15), (uint8_t)(posToSend >> 7), 
                         (uint8_t)(indexToSend >> 15),(uint8_t)(indexToSend >> 7), 
                         sign_pos | (0b01111111 & posToSend), sign_index | (0b01111111 & indexToSend), 0);
        // if(debug_last_pos < encoder_value)
        //     can_transmit(15, encoder_value - debug_last_pos, 0,0,0,130,244,0);
        // else
        //     can_transmit(16, debug_last_pos - encoder_value, 0,0,0,240,230,0);
        // debug_last_pos = encoder_value;
    }
    //uart_analyse_buffer();
    
    if (mode == VELOCITY)
    {
        target_position += velocity * CONTROL_LOOP_PERIOD;
        velocity_decimal_remainder += velocity * CONTROL_LOOP_PERIOD;
        if (velocity_decimal_remainder < 0)
        {
            target_position -= (int)(-velocity_decimal_remainder);
            velocity_decimal_remainder += (int)(-velocity_decimal_remainder);
        }
        else
        {
            target_position += (int)(velocity_decimal_remainder);
            velocity_decimal_remainder -= (int)(velocity_decimal_remainder);
        }
    }

    if (mode == VELOCITY || mode == POSITION)
        tick(pid);
    else if (mode == POWER)
        set_power(power);

    if (HAL_GetTick() - watchdog_last_time > watchdog)
        mode = STOPPED;

    if (mode == POWER)
    {
        set_power(power);
    }

    if (mode == STOPPED)
        set_power(0);
}

void pwm_setvalue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
  
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  
}

float pid_source()
{
    return (float)(encoder_value - target_position) * step_size;
}

void set_power(float power) // -1 to 1 range
{
    last_power = power;
    static int16_t pwm;
    static uint8_t dir;
    if (power < -1)
        power = -1;
    if (power > 1)
        power = 1;
    pwm = abs(power) * MAX_PWM;
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, !!pwm);
    dir = (power > 0);
    if (count_control % 10 == 0)
    {
        //can_transmit(20, pwm / 100, pwm % 100, 0,0,0,0,0);
    }
    pwm_setvalue(pwm);
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, dir);
}

// void uart_byte_received(uint8_t byte)
// {
//     uart_buffer[uart_count] = byte;
//     uart_count++;
//     uart_count %= UART_BUFFER_SIZE;
//     //uart_transmit("d\r\n", 3);
// }

// void can_transmit(uint8_t *data)
// {
//     TxHeader.StdId = 1<<10 | DEVICE_ID;
//     TxHeader.RTR = CAN_RTR_DATA;
//     TxHeader.IDE = CAN_ID_STD;
//     TxHeader.DLC = 8;
//     TxHeader.TransmitGlobalTime = DISABLE;
//     HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
// }
HAL_StatusTypeDef can_transmit(uint8_t header, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
    TxHeader.StdId = 1<<10 | DEVICE_ID;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    static uint8_t data[8];
    data[0] = header;
    data[1] = d1;
    data[2] = d2;
    data[3] = d3;
    data[4] = d4;
    data[5] = d5;
    data[6] = d6;
    data[7] = d7;
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data,  &TxMailbox);
}

void parse_can_message()
{
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
    if ((int32_t)RxHeader.StdId == DEVICE_ID)
    {
        if (RxData[0] == 18)
        {
            if (RxData[1] == 0)
                position_info_period_ms = 0;
            else
            {
                position_info_period_ms = 1000 / RxData[1];
                if (position_info_period_ms < 10)
                    position_info_period_ms = 10;
            } 
        }
        else if (RxData[0] == 7) // set_power
        {
            power = (float)((int16_t)(((uint16_t)RxData[1] << 8) | RxData[2])) * (1.0 / 256 / 128);
            mode = POWER;
            watchdog_last_time = HAL_GetTick();
        }
        else if (RxData[0] == 8) // set_target_position
        {
            target_position = (float)((int16_t)(((uint16_t)RxData[1] << 8) | RxData[2])) / step_size;
            mode = POSITION;
            watchdog_last_time = HAL_GetTick();
        }
        else if (RxData[0] == 38) // set_encoder_position
        {
            int32_t new_encoder_value = (float)(((int16_t)RxData[1] << 8) | RxData[2]) / step_size;
            int32_t new_index_pulse_position = 
                index_pulse_position + new_encoder_value - encoder_value;
            if (!rotational_encoder || (new_index_pulse_position > 0 && new_index_pulse_position <= 3600 / step_size))
                index_pulse_position = new_index_pulse_position;
            else
                index_pulse_position = 0xffff;
            encoder_value = new_encoder_value;
        }
        else if (RxData[0] == 35) // set P parameter
        {
            int32_t int_data = (RxData[1]<<8) | RxData[2];
            int_data = int_data << 16 | ((RxData[3] << 8) | RxData[4]);
            pid->p = int_data * 0.0001f * step_size;
        }
        else if (RxData[0] == 36) // set I parameter
        {
            int32_t int_data = (RxData[1]<<8) | RxData[2];
            int_data = int_data << 16 | ((RxData[3] << 8) | RxData[4]);
            pid->i = int_data * 0.0001f * step_size;
        }
        else if (RxData[0] == 37) // set D parameter
        {
            int32_t int_data = (RxData[1]<<8) | RxData[2];
            int_data = int_data << 16 | ((RxData[3] << 8) | RxData[4]);
            pid->d = int_data * 0.0001f * step_size;
        }

        // } else if (m->header == 35) { //P parameter
        // int32_t int_data = m->payload[0]<<8 | m->payload[1];
        // int_data = int_data << 16 | (m->payload[2]<<8 | m->payload[3]);
        // positionPid.Kp = int_data / 10000.0f;
        // } else if (m->header == 36) { //I parameter
        //     int32_t int_data = m->payload[0]<<8 | m->payload[1];
        //     int_data = int_data << 16 | (m->payload[2]<<8 | m->payload[3]);
        //     positionPid.Ki = int_data / 10000.0f;
        // } else if (m->header == 37) { //D parameter
        //     int32_t int_data = m->payload[0]<<8 | m->payload[1];
        //     int_data = int_data << 16 | (m->payload[2]<<8 | m->payload[3]);
        //     positionPid.Kd = int_data / 10000.0f;
        // }
        // else if (RxData[0] == 9) // set_velocity
        // {
        //     target_position = (float)(((int16_t)(RxData[1] & 0b01111111) << 8) | RxData[2]) / step_size ;
        //     if (RxData[1] & 0b10000000)
        //         target_position *= -1;
        //     mode = POSITION;
        //     watchdog_last_time = HAL_GetTick();
        // }
    }
    //uart_transmit(buffer, count);
}

// void check_tx_buffer()
// {
//     if (tx_busy)
//         return; // sending, this function will be executed again at tx finish callback
//     uint8_t end = uart_tx_buffer_end; // i make a copy in case of value change
//     if (end > uart_tx_buffer_begin) // ordinary send, no circular buffer overflow
//     {
//         tx_busy = 1;
//         HAL_UART_Transmit_DMA(&huart2, (unsigned char*)(uart_tx_buffer + uart_tx_buffer_begin), end - uart_tx_buffer_begin);
//         uart_tx_buffer_begin = end;
//     }
//     else if (end < uart_tx_buffer_begin)// circular buffer overflow, now send only till the end of buffer
//     {
//         tx_busy = 1;
//         HAL_UART_Transmit_DMA(&huart2, (unsigned char*)(uart_tx_buffer + uart_tx_buffer_begin), UART_BUFFER_SIZE - uart_tx_buffer_begin);
//         uart_tx_buffer_begin = 0;
//     }

// }

// void tx_cplt()
// {
//     tx_busy = 0;
//     check_tx_buffer();
// }

// void uart_transmit(uint8_t *data, uint16_t len)
// {
//     HAL_UART_Transmit(&huart2, data, len, 500);
//     return;
//     uint8_t i;
//     for (i = 0; i < len; i++)
//     {
//         uart_tx_buffer[uart_tx_buffer_end] = data[i];
//         uart_tx_buffer_end = (uart_tx_buffer_end + 1) % UART_BUFFER_SIZE;
//     }
//     check_tx_buffer();
// }

// void uart_analyse_buffer()
// {
//     uint8_t uart_current_count;
//     uint8_t i;
//     uint16_t command_signature;

//     uart_current_count = uart_count;
//     i = uart_last_count;

//     while (i != uart_current_count)
//     {   
//         uart_message[uart_message_len++] = uart_buffer[i++];
//         i %= UART_BUFFER_SIZE;
//         uart_last_count = i;
//         uint8_t byte = uart_message[uart_message_len - 1];
//         if (byte == '\r' && uart_message_len > 1)
//         {
//             uart_message[uart_message_len] = 0;
//             uint8_t j = 0;

//             while (uart_message[j] == ' ' || uart_message[j] == '\t' || uart_message[j] == '\r' || uart_message[j] == '\n') // trim whitespace
//             {
//                 j++;
//                 if (j + 2 >= uart_message_len) // must be at least three characters left - two for command and one for endline
//                 {
//                     uart_message_len = 0;
//                     HAL_UART_Transmit(&huart2, (unsigned char*)"?\r", 2, 500);
//                     //uart_analyse_buffer();
//                     return;
//                 }
//             }

//             float value = strtof((char *)(uart_message + j + 2), NULL); // try to read numerical value from part of buffer two bytes ahead

//             command_signature = 0x0100 * uart_message[j] + uart_message[j+1];

//            switch(command_signature)
//             {
//                 case 0x0100 * 'm' + 'p':  
//                     mode = POWER;                
//                     power = value;
//                     count = sprintf((char*)buffer, "mp\r");
//                     HAL_UART_Transmit(&huart2, buffer, count, 500);
//                     break; 
//                 case 0x0100 * 'm' + 'a':  
//                     mode = POSITION;                
//                     target_position = value / step_size;
//                     count = sprintf((char*)buffer, "ma\r");
//                     HAL_UART_Transmit(&huart2, buffer, count, 500);
//                     break; 
//                 case 0x0100 * 'm' + 'v':          
//                     target_position = encoder_value;
//                     mode = VELOCITY;  
//                     velocity = value / step_size;    
//                     count = sprintf((char*)buffer, "mv\r");
//                     HAL_UART_Transmit(&huart2, buffer, count, 500);
//                     break; 
//                 case 0x0100 * 'm' + 'r':  
//                     if (mode == POSITION)
//                         target_position += value / step_size;
//                     else
//                     {
//                         target_position = encoder_value + value / step_size;
//                         mode = POSITION;
//                     }
//                     count = sprintf((char*)buffer, "mr\r");
//                     HAL_UART_Transmit(&huart2, buffer, count, 500);
//                     break; 
//                 /*case 0x0100 * 's' + 's':  
//                     step
//                     count = sprintf((char*)buffer, "ss\r");
//                     HAL_UART_Transmit(&huart2, buffer, count, 500);
//                     break;*/ 
//                 default:
//                     HAL_UART_Transmit(&huart2, (unsigned char*)"?\r", 2, 500);
//             }
//             uart_message_len = 0;
//             return;
//         }
//     }
// }

int usrMain() 
{  
    pid = createPIDController(0.006, 0, 0*0.0005, pid_source, set_power);
    //pid->maxOutputSweep = 15.0f;
    pid->maxCumulation = 200;
    pid->deadZone = 0.025;
    pid->deltaTime = CONTROL_LOOP_PERIOD;
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    while(1)
    {
        
    }

    return 0;
}
