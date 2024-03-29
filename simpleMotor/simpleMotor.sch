EESchema Schematic File Version 4
LIBS:simpleMotor-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L simpleMotor-rescue:step_down_mp1584-d_sun U3
U 1 1 5D665108
P 4850 2650
F 0 "U3" H 5350 2650 50  0000 C CNN
F 1 "step_down_mp1584" H 5150 2950 50  0000 C CNN
F 2 "d-sun:mp1584" H 4850 2650 50  0001 C CNN
F 3 "" H 4850 2650 50  0001 C CNN
	1    4850 2650
	-1   0    0    1   
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR011
U 1 1 5D6677FD
P 4850 2650
F 0 "#PWR011" H 4850 2400 50  0001 C CNN
F 1 "GND" V 4855 2522 50  0000 R CNN
F 2 "" H 4850 2650 50  0001 C CNN
F 3 "" H 4850 2650 50  0001 C CNN
	1    4850 2650
	0    -1   -1   0   
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR03
U 1 1 5D6688A0
P 3300 2100
F 0 "#PWR03" H 3300 1850 50  0001 C CNN
F 1 "GND" V 3305 1972 50  0000 R CNN
F 2 "" H 3300 2100 50  0001 C CNN
F 3 "" H 3300 2100 50  0001 C CNN
	1    3300 2100
	0    -1   -1   0   
$EndComp
$Comp
L simpleMotor-rescue:NUCLEO-L432KC-STM32L432KCU6 U1
U 1 1 5D6545AB
P 2600 2700
F 0 "U1" H 2600 3720 50  0000 C CNN
F 1 "NUCLEO-L432KC" H 2600 3629 50  0000 C CNN
F 2 "STM32:NUCLEO-L432KC" H 2550 3800 50  0001 L BNN
F 3 "STMicroelectronics" H 2550 3700 50  0001 L BNN
F 4 "STM32L432KCU6" H 2550 3600 50  0001 L BNN "Pole8"
	1    2600 2700
	1    0    0    -1  
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR01
U 1 1 5D669120
P 1900 2300
F 0 "#PWR01" H 1900 2050 50  0001 C CNN
F 1 "GND" V 1905 2172 50  0000 R CNN
F 2 "" H 1900 2300 50  0001 C CNN
F 3 "" H 1900 2300 50  0001 C CNN
	1    1900 2300
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:Conn_01x02-Connector_Generic J9
U 1 1 5D669DF5
P 8200 3250
F 0 "J9" H 8280 3242 50  0000 L CNN
F 1 "BATTERY_IN" H 8280 3151 50  0000 L CNN
F 2 "XT:XT30_vertical" H 8200 3250 50  0001 C CNN
F 3 "~" H 8200 3250 50  0001 C CNN
	1    8200 3250
	1    0    0    -1  
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR021
U 1 1 5D66A885
P 8000 3250
F 0 "#PWR021" H 8000 3000 50  0001 C CNN
F 1 "GND" V 8005 3122 50  0000 R CNN
F 2 "" H 8000 3250 50  0001 C CNN
F 3 "" H 8000 3250 50  0001 C CNN
	1    8000 3250
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:+7.5V-power #PWR02
U 1 1 5D66B593
P 3300 2000
F 0 "#PWR02" H 3300 1850 50  0001 C CNN
F 1 "+7.5V" V 3315 2128 50  0000 L CNN
F 2 "" H 3300 2000 50  0001 C CNN
F 3 "" H 3300 2000 50  0001 C CNN
	1    3300 2000
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:+7.5V-power #PWR012
U 1 1 5D66CA32
P 4850 2800
F 0 "#PWR012" H 4850 2650 50  0001 C CNN
F 1 "+7.5V" V 4865 2928 50  0000 L CNN
F 2 "" H 4850 2800 50  0001 C CNN
F 3 "" H 4850 2800 50  0001 C CNN
	1    4850 2800
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:+24V-power #PWR022
U 1 1 5D66D5C5
P 8000 3350
F 0 "#PWR022" H 8000 3200 50  0001 C CNN
F 1 "+24V" V 8015 3478 50  0000 L CNN
F 2 "" H 8000 3350 50  0001 C CNN
F 3 "" H 8000 3350 50  0001 C CNN
	1    8000 3350
	0    -1   -1   0   
$EndComp
$Comp
L simpleMotor-rescue:+5V-power #PWR04
U 1 1 5D66F37D
P 3300 2300
F 0 "#PWR04" H 3300 2150 50  0001 C CNN
F 1 "+5V" V 3315 2428 50  0000 L CNN
F 2 "" H 3300 2300 50  0001 C CNN
F 3 "" H 3300 2300 50  0001 C CNN
	1    3300 2300
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:+3V3-power #PWR05
U 1 1 5D66FFFA
P 3300 3300
F 0 "#PWR05" H 3300 3150 50  0001 C CNN
F 1 "+3V3" V 3315 3428 50  0000 L CNN
F 2 "" H 3300 3300 50  0001 C CNN
F 3 "" H 3300 3300 50  0001 C CNN
	1    3300 3300
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:Conn_01x08-Connector_Generic J3
U 1 1 5D672E7A
P 5600 3450
F 0 "J3" H 5680 3442 50  0000 L CNN
F 1 "POLOLU_SIG" H 5680 3351 50  0000 L CNN
F 2 "pololu:Pin_Header_Straight_1x08_Pitch2.54mm" H 5600 3450 50  0001 C CNN
F 3 "~" H 5600 3450 50  0001 C CNN
	1    5600 3450
	1    0    0    -1  
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR013
U 1 1 5D673B3C
P 5400 3150
F 0 "#PWR013" H 5400 2900 50  0001 C CNN
F 1 "GND" V 5405 3022 50  0000 R CNN
F 2 "" H 5400 3150 50  0001 C CNN
F 3 "" H 5400 3150 50  0001 C CNN
	1    5400 3150
	0    1    1    0   
$EndComp
Text GLabel 5400 3250 0    50   Input ~ 0
DIR
Text GLabel 5400 3350 0    50   Input ~ 0
PWM
Text GLabel 5400 3650 0    50   Input ~ 0
CS
Text GLabel 5400 3750 0    50   Input ~ 0
3v3ref
Text GLabel 5400 3850 0    50   Input ~ 0
VM
Text GLabel 4150 2800 0    50   Input ~ 0
VM
NoConn ~ 5400 3450
NoConn ~ 5400 3550
$Comp
L simpleMotor-rescue:Conn_01x04-Connector_Generic J6
U 1 1 5D674F90
P 6800 3450
F 0 "J6" H 6718 3025 50  0000 C CNN
F 1 "POLOLU_POW" H 6718 3116 50  0000 C CNN
F 2 "pololu:Pin_Header_Pololu_OUT" H 6800 3450 50  0001 C CNN
F 3 "~" H 6800 3450 50  0001 C CNN
	1    6800 3450
	-1   0    0    1   
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR018
U 1 1 5D67578F
P 7000 3550
F 0 "#PWR018" H 7000 3300 50  0001 C CNN
F 1 "GND" V 7005 3422 50  0000 R CNN
F 2 "" H 7000 3550 50  0001 C CNN
F 3 "" H 7000 3550 50  0001 C CNN
	1    7000 3550
	0    -1   -1   0   
$EndComp
$Comp
L simpleMotor-rescue:+24V-power #PWR017
U 1 1 5D6760A2
P 7000 3250
F 0 "#PWR017" H 7000 3100 50  0001 C CNN
F 1 "+24V" V 7015 3378 50  0000 L CNN
F 2 "" H 7000 3250 50  0001 C CNN
F 3 "" H 7000 3250 50  0001 C CNN
	1    7000 3250
	0    1    1    0   
$EndComp
Text GLabel 7000 3450 2    50   Input ~ 0
MOTOR_A
Text GLabel 7000 3350 2    50   Input ~ 0
MOTOR_B
$Comp
L simpleMotor-rescue:Conn_01x02-Connector_Generic J10
U 1 1 5D678D7E
P 8200 3500
F 0 "J10" H 8280 3492 50  0000 L CNN
F 1 "MOTOR_OUT" H 8280 3401 50  0000 L CNN
F 2 "XT:XT30_vertical" H 8200 3500 50  0001 C CNN
F 3 "~" H 8200 3500 50  0001 C CNN
	1    8200 3500
	1    0    0    -1  
$EndComp
Text GLabel 8000 3500 0    50   Input ~ 0
MOTOR_B
Text GLabel 8000 3600 0    50   Input ~ 0
MOTOR_A
$Comp
L simpleMotor-rescue:Conn_01x01-Connector_Generic J8
U 1 1 5D6799A0
P 8200 3000
F 0 "J8" H 8280 3042 50  0000 L CNN
F 1 "CAP+" H 8280 2951 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1.2mm" H 8200 3000 50  0001 C CNN
F 3 "~" H 8200 3000 50  0001 C CNN
	1    8200 3000
	1    0    0    -1  
$EndComp
$Comp
L simpleMotor-rescue:Conn_01x01-Connector_Generic J7
U 1 1 5D67A60E
P 8200 2850
F 0 "J7" H 8280 2892 50  0000 L CNN
F 1 "CAP-" H 8280 2801 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1.2mm" H 8200 2850 50  0001 C CNN
F 3 "~" H 8200 2850 50  0001 C CNN
	1    8200 2850
	1    0    0    -1  
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR019
U 1 1 5D67A8AD
P 8000 2850
F 0 "#PWR019" H 8000 2600 50  0001 C CNN
F 1 "GND" V 8005 2722 50  0000 R CNN
F 2 "" H 8000 2850 50  0001 C CNN
F 3 "" H 8000 2850 50  0001 C CNN
	1    8000 2850
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:+24V-power #PWR020
U 1 1 5D67AFE1
P 8000 3000
F 0 "#PWR020" H 8000 2850 50  0001 C CNN
F 1 "+24V" V 8015 3128 50  0000 L CNN
F 2 "" H 8000 3000 50  0001 C CNN
F 3 "" H 8000 3000 50  0001 C CNN
	1    8000 3000
	0    -1   -1   0   
$EndComp
$Comp
L simpleMotor-rescue:Conn_01x06-Connector_Generic J1
U 1 1 5D67B442
P 4850 2100
F 0 "J1" H 4930 2092 50  0000 L CNN
F 1 "BOTLAND_MOTOR" H 4930 2001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Horizontal" H 4850 2100 50  0001 C CNN
F 3 "~" H 4850 2100 50  0001 C CNN
	1    4850 2100
	1    0    0    -1  
$EndComp
Text GLabel 4650 1900 0    50   Input ~ 0
MOTOR_B
Text GLabel 4650 2000 0    50   Input ~ 0
MOTOR_A
$Comp
L simpleMotor-rescue:GND-power #PWR09
U 1 1 5D67CFC9
P 4650 2100
F 0 "#PWR09" H 4650 1850 50  0001 C CNN
F 1 "GND" V 4655 1972 50  0000 R CNN
F 2 "" H 4650 2100 50  0001 C CNN
F 3 "" H 4650 2100 50  0001 C CNN
	1    4650 2100
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:+5V-power #PWR010
U 1 1 5D67D8AA
P 4650 2200
F 0 "#PWR010" H 4650 2050 50  0001 C CNN
F 1 "+5V" V 4665 2328 50  0000 L CNN
F 2 "" H 4650 2200 50  0001 C CNN
F 3 "" H 4650 2200 50  0001 C CNN
	1    4650 2200
	0    -1   -1   0   
$EndComp
Text GLabel 4650 2300 0    50   Input ~ 0
ENC_A
Text GLabel 4650 2400 0    50   Input ~ 0
ENC_B
$Comp
L simpleMotor-rescue:Conn_01x07-Connector_Generic J4
U 1 1 5D67EF84
P 6100 2300
F 0 "J4" H 6180 2342 50  0000 L CNN
F 1 "ENCODER" H 6180 2251 50  0000 L CNN
F 2 "Connector_JST:JST_PH_S7B-PH-SM4-TB_1x07-1MP_P2.00mm_Horizontal" H 6100 2300 50  0001 C CNN
F 3 "~" H 6100 2300 50  0001 C CNN
	1    6100 2300
	1    0    0    -1  
$EndComp
$Comp
L simpleMotor-rescue:+5V-power #PWR014
U 1 1 5D680875
P 5900 2000
F 0 "#PWR014" H 5900 1850 50  0001 C CNN
F 1 "+5V" V 5915 2128 50  0000 L CNN
F 2 "" H 5900 2000 50  0001 C CNN
F 3 "" H 5900 2000 50  0001 C CNN
	1    5900 2000
	0    -1   -1   0   
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR016
U 1 1 5D6815D7
P 5900 2600
F 0 "#PWR016" H 5900 2350 50  0001 C CNN
F 1 "GND" V 5905 2472 50  0000 R CNN
F 2 "" H 5900 2600 50  0001 C CNN
F 3 "" H 5900 2600 50  0001 C CNN
	1    5900 2600
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:+3V3-power #PWR015
U 1 1 5D681C5D
P 5900 2400
F 0 "#PWR015" H 5900 2250 50  0001 C CNN
F 1 "+3V3" V 5915 2528 50  0000 L CNN
F 2 "" H 5900 2400 50  0001 C CNN
F 3 "" H 5900 2400 50  0001 C CNN
	1    5900 2400
	0    -1   -1   0   
$EndComp
Text GLabel 5900 2500 0    50   Input ~ 0
ANALOG_ENC
Text GLabel 5900 2100 0    50   Input ~ 0
ENC_A
Text GLabel 5900 2200 0    50   Input ~ 0
ENC_B
Text GLabel 5900 2300 0    50   Input ~ 0
ENC_I
$Comp
L can:SN65HVD231 U2
U 1 1 5D683995
P 4250 4450
F 0 "U2" H 4250 5097 60  0000 C CNN
F 1 "SN65HVD231" H 4250 4991 60  0000 C CNN
F 2 "Package_SO:SO-8_5.3x6.2mm_P1.27mm" H 4250 4908 30  0000 C CNN
F 3 "" H 4250 4450 60  0000 C CNN
	1    4250 4450
	1    0    0    -1  
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR08
U 1 1 5D685C42
P 3750 4750
F 0 "#PWR08" H 3750 4500 50  0001 C CNN
F 1 "GND" V 3755 4622 50  0000 R CNN
F 2 "" H 3750 4750 50  0001 C CNN
F 3 "" H 3750 4750 50  0001 C CNN
	1    3750 4750
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR07
U 1 1 5D6860F9
P 3750 4450
F 0 "#PWR07" H 3750 4200 50  0001 C CNN
F 1 "GND" V 3755 4322 50  0000 R CNN
F 2 "" H 3750 4450 50  0001 C CNN
F 3 "" H 3750 4450 50  0001 C CNN
	1    3750 4450
	0    1    1    0   
$EndComp
NoConn ~ 3750 4250
$Comp
L simpleMotor-rescue:+3V3-power #PWR06
U 1 1 5D686691
P 3750 4150
F 0 "#PWR06" H 3750 4000 50  0001 C CNN
F 1 "+3V3" V 3765 4278 50  0000 L CNN
F 2 "" H 3750 4150 50  0001 C CNN
F 3 "" H 3750 4150 50  0001 C CNN
	1    3750 4150
	0    -1   -1   0   
$EndComp
Text GLabel 1900 2500 0    50   Input ~ 0
ENC_A
Text GLabel 1900 2800 0    50   Input ~ 0
ENC_B
Text GLabel 1900 3400 0    50   Input ~ 0
ENC_I
Text GLabel 3300 2600 2    50   Input ~ 0
CS
Text GLabel 3750 4550 0    50   Input ~ 0
CAN_TX
Text GLabel 3750 4650 0    50   Input ~ 0
CAN_RX
Text GLabel 1900 2400 0    50   Input ~ 0
CAN_TX
Text GLabel 1900 3200 0    50   Input ~ 0
CAN_RX
Text GLabel 3300 2500 2    50   Input ~ 0
ANALOG_ENC
Text GLabel 3300 2900 2    50   Input ~ 0
PWM
Text GLabel 3300 2800 2    50   Input ~ 0
DIR
Text Notes 2000 3650 0    50   ~ 0
REMOVE JUMPERS SB16, SB18
$Comp
L simpleMotor-rescue:Conn_01x04-Connector_Generic J2
U 1 1 5D689658
P 5250 4400
F 0 "J2" H 5330 4392 50  0000 L CNN
F 1 "CAN1" H 5150 4050 50  0000 L CNN
F 2 "Connector_JST:JST_PH_S4B-PH-SM4-TB_1x04-1MP_P2.00mm_Horizontal" H 5250 4400 50  0001 C CNN
F 3 "~" H 5250 4400 50  0001 C CNN
	1    5250 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4750 4750 4500
Wire Wire Line
	4750 4500 5050 4500
Wire Wire Line
	5050 4400 4750 4400
Wire Wire Line
	4750 4400 4750 4150
$Comp
L simpleMotor-rescue:Conn_01x04-Connector_Generic J5
U 1 1 5D68ACFD
P 6200 4400
F 0 "J5" H 6280 4392 50  0000 L CNN
F 1 "CAN2" H 6100 4050 50  0000 L CNN
F 2 "Connector_JST:JST_PH_S4B-PH-SM4-TB_1x04-1MP_P2.00mm_Horizontal" H 6200 4400 50  0001 C CNN
F 3 "~" H 6200 4400 50  0001 C CNN
	1    6200 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4400 6000 4400
Connection ~ 5050 4400
Wire Wire Line
	6000 4500 5050 4500
Connection ~ 5050 4500
Wire Wire Line
	5050 4600 6000 4600
Wire Wire Line
	6000 4300 5050 4300
$Comp
L simpleMotor-rescue:LED-Device D1
U 1 1 5D69D627
P 2200 4050
F 0 "D1" H 2193 4266 50  0000 C CNN
F 1 "RED" H 2193 4175 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 2200 4050 50  0001 C CNN
F 3 "~" H 2200 4050 50  0001 C CNN
	1    2200 4050
	1    0    0    -1  
$EndComp
$Comp
L simpleMotor-rescue:R-Device R1
U 1 1 5D69E3AD
P 2500 4050
F 0 "R1" V 2293 4050 50  0000 C CNN
F 1 "R" V 2384 4050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2430 4050 50  0001 C CNN
F 3 "~" H 2500 4050 50  0001 C CNN
	1    2500 4050
	0    1    1    0   
$EndComp
Text GLabel 2650 4050 2    50   Input ~ 0
LED_R
$Comp
L simpleMotor-rescue:LED-Device D2
U 1 1 5D6A557B
P 2200 4350
F 0 "D2" H 2193 4566 50  0000 C CNN
F 1 "YELLOW" H 2193 4475 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 2200 4350 50  0001 C CNN
F 3 "~" H 2200 4350 50  0001 C CNN
	1    2200 4350
	1    0    0    -1  
$EndComp
$Comp
L simpleMotor-rescue:R-Device R2
U 1 1 5D6A5585
P 2500 4350
F 0 "R2" V 2293 4350 50  0000 C CNN
F 1 "R" V 2384 4350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2430 4350 50  0001 C CNN
F 3 "~" H 2500 4350 50  0001 C CNN
	1    2500 4350
	0    1    1    0   
$EndComp
Text GLabel 2650 4350 2    50   Input ~ 0
LED_YELLOW
$Comp
L simpleMotor-rescue:LED-Device D3
U 1 1 5D6A8E57
P 2200 4650
F 0 "D3" H 2193 4866 50  0000 C CNN
F 1 "BLUE" H 2193 4775 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 2200 4650 50  0001 C CNN
F 3 "~" H 2200 4650 50  0001 C CNN
	1    2200 4650
	1    0    0    -1  
$EndComp
$Comp
L simpleMotor-rescue:R-Device R3
U 1 1 5D6A8E61
P 2500 4650
F 0 "R3" V 2293 4650 50  0000 C CNN
F 1 "R" V 2384 4650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2430 4650 50  0001 C CNN
F 3 "~" H 2500 4650 50  0001 C CNN
	1    2500 4650
	0    1    1    0   
$EndComp
Text GLabel 2650 4650 2    50   Input ~ 0
LED_BLUE
$Comp
L simpleMotor-rescue:GND-power #PWR0101
U 1 1 5D666103
P 2050 4050
F 0 "#PWR0101" H 2050 3800 50  0001 C CNN
F 1 "GND" V 2055 3922 50  0000 R CNN
F 2 "" H 2050 4050 50  0001 C CNN
F 3 "" H 2050 4050 50  0001 C CNN
	1    2050 4050
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR0102
U 1 1 5D6665F6
P 2050 4350
F 0 "#PWR0102" H 2050 4100 50  0001 C CNN
F 1 "GND" V 2055 4222 50  0000 R CNN
F 2 "" H 2050 4350 50  0001 C CNN
F 3 "" H 2050 4350 50  0001 C CNN
	1    2050 4350
	0    1    1    0   
$EndComp
$Comp
L simpleMotor-rescue:GND-power #PWR0103
U 1 1 5D66690F
P 2050 4650
F 0 "#PWR0103" H 2050 4400 50  0001 C CNN
F 1 "GND" V 2055 4522 50  0000 R CNN
F 2 "" H 2050 4650 50  0001 C CNN
F 3 "" H 2050 4650 50  0001 C CNN
	1    2050 4650
	0    1    1    0   
$EndComp
Text GLabel 3300 3000 2    50   Input ~ 0
LED_R
Text GLabel 3300 2700 2    50   Input ~ 0
LED_YELLOW
Text GLabel 1900 3100 0    50   Input ~ 0
LED_BLUE
NoConn ~ 3300 2200
NoConn ~ 3300 2400
NoConn ~ 1900 2000
NoConn ~ 1900 2100
NoConn ~ 1900 2200
NoConn ~ 1900 3300
NoConn ~ 3300 3100
NoConn ~ 3300 3200
NoConn ~ 3300 3400
Text GLabel 1900 2600 0    50   Input ~ 0
ENC_A
Text GLabel 1900 2700 0    50   Input ~ 0
ENC_B
$EndSCHEMATC
