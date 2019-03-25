EESchema Schematic File Version 4
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "TPMS Silencer"
Date ""
Rev "v2"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	5000 1950 4800 1950
Text Label 4800 2050 0    50   ~ 0
ASK
Wire Wire Line
	5000 2050 4800 2050
Text Label 4800 1950 0    50   ~ 0
FSK
$Comp
L power:GNDREF #PWR08
U 1 1 5BBBC293
P 5300 3050
F 0 "#PWR08" H 5300 2800 50  0001 C CNN
F 1 "GNDREF" H 5305 2877 50  0000 C CNN
F 2 "" H 5300 3050 50  0001 C CNN
F 3 "" H 5300 3050 50  0001 C CNN
	1    5300 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 3050 5300 3050
Connection ~ 5300 3050
$Comp
L Device:C_Small C6
U 1 1 5BBBC3E8
P 3900 2550
F 0 "C6" V 4100 2550 50  0000 C CNN
F 1 "10pF" V 4000 2500 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3900 2550 50  0001 C CNN
F 3 "" H 3900 2550 50  0001 C CNN
	1    3900 2550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5000 2550 4000 2550
$Comp
L Device:C_Small C5
U 1 1 5BBBC557
P 3500 2150
F 0 "C5" V 3271 2150 50  0000 C CNN
F 1 "8pF" V 3362 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3500 2150 50  0001 C CNN
F 3 "" H 3500 2150 50  0001 C CNN
	1    3500 2150
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR07
U 1 1 5BBBC59F
P 3300 2150
F 0 "#PWR07" H 3300 1900 50  0001 C CNN
F 1 "GNDREF" H 3450 2000 50  0000 R CNN
F 2 "" H 3300 2150 50  0001 C CNN
F 3 "" H 3300 2150 50  0001 C CNN
	1    3300 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal_Small Y2
U 1 1 5BBBC734
P 3800 1950
F 0 "Y2" V 3754 2038 50  0000 L CNN
F 1 "9.84375MHz" V 3845 2038 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_HC49-SD" H 3800 1950 50  0001 C CNN
F 3 "" H 3800 1950 50  0001 C CNN
	1    3800 1950
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5BBBC7B7
P 3500 1750
F 0 "C4" V 3271 1750 50  0000 C CNN
F 1 "8pF" V 3362 1750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3500 1750 50  0001 C CNN
F 3 "" H 3500 1750 50  0001 C CNN
	1    3500 1750
	0    1    1    0   
$EndComp
Wire Wire Line
	3600 1750 3800 1750
Wire Wire Line
	3800 1750 3800 1850
Wire Wire Line
	3800 2050 3800 2150
Wire Wire Line
	3800 2150 3600 2150
Wire Wire Line
	3400 2150 3300 2150
Wire Wire Line
	3300 2150 3300 1750
Wire Wire Line
	3300 1750 3400 1750
Connection ~ 3300 2150
Wire Wire Line
	4500 1750 3800 1750
Connection ~ 3800 1750
Wire Wire Line
	3800 2550 3800 2250
Connection ~ 3800 2150
Wire Wire Line
	5000 2350 4400 2350
Wire Wire Line
	4400 2350 4400 2250
Wire Wire Line
	4400 2250 3800 2250
Connection ~ 3800 2250
Wire Wire Line
	3800 2250 3800 2150
Wire Wire Line
	4500 1750 4500 2250
Text Label 6200 1950 2    50   ~ 0
EN
$Comp
L Device:L L1
U 1 1 5BBBDA7E
P 6600 1950
F 0 "L1" H 6559 1904 50  0000 R CNN
F 1 "470nH" H 6559 1995 50  0000 R CNN
F 2 "Inductor_SMD:L_0805_2012Metric" H 6600 1950 50  0001 C CNN
F 3 "" H 6600 1950 50  0001 C CNN
	1    6600 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	6600 2250 6600 2100
$Comp
L Device:C_Small C8
U 1 1 5BBBDE9F
P 7050 1900
F 0 "C8" H 7142 1946 50  0000 L CNN
F 1 "4.7uF 50V" H 7142 1855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7050 1900 50  0001 C CNN
F 3 "" H 7050 1900 50  0001 C CNN
	1    7050 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR011
U 1 1 5BBBE0CF
P 7050 2000
F 0 "#PWR011" H 7050 1750 50  0001 C CNN
F 1 "GNDREF" H 7055 1827 50  0000 C CNN
F 2 "" H 7050 2000 50  0001 C CNN
F 3 "" H 7050 2000 50  0001 C CNN
	1    7050 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 1800 7050 1600
Wire Wire Line
	7050 1600 6600 1600
Wire Wire Line
	6600 1600 6600 1800
$Comp
L Device:R_Small R3
U 1 1 5BBBE7CB
P 6600 1450
F 0 "R3" H 6659 1496 50  0000 L CNN
F 1 "0" H 6659 1405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 6600 1450 50  0001 C CNN
F 3 "" H 6600 1450 50  0001 C CNN
	1    6600 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 1600 6600 1550
Connection ~ 6600 1600
$Comp
L Device:C_Small C7
U 1 1 5BBBECCB
P 6900 1100
F 0 "C7" H 6992 1146 50  0000 L CNN
F 1 "100pF 50V" H 6992 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6900 1100 50  0001 C CNN
F 3 "" H 6900 1100 50  0001 C CNN
	1    6900 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 5BBBECFD
P 7550 1100
F 0 "C10" H 7642 1146 50  0000 L CNN
F 1 "10uF 6.3V" H 7642 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7550 1100 50  0001 C CNN
F 3 "" H 7550 1100 50  0001 C CNN
	1    7550 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C12
U 1 1 5BBBED2F
P 8200 1100
F 0 "C12" H 8292 1146 50  0000 L CNN
F 1 "0.1uF 50V" H 8292 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8200 1100 50  0001 C CNN
F 3 "" H 8200 1100 50  0001 C CNN
	1    8200 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 1000 8200 1000
Connection ~ 6900 1000
Wire Wire Line
	6900 1000 6600 1000
$Comp
L power:GNDREF #PWR013
U 1 1 5BBBF1D2
P 7550 1300
F 0 "#PWR013" H 7550 1050 50  0001 C CNN
F 1 "GNDREF" H 7555 1127 50  0000 C CNN
F 2 "" H 7550 1300 50  0001 C CNN
F 3 "" H 7550 1300 50  0001 C CNN
	1    7550 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 1250 6900 1200
$Comp
L power:VDD #PWR09
U 1 1 5BBC0218
P 5400 1650
F 0 "#PWR09" H 5400 1500 50  0001 C CNN
F 1 "VDD" H 5417 1823 50  0000 C CNN
F 2 "" H 5400 1650 50  0001 C CNN
F 3 "" H 5400 1650 50  0001 C CNN
	1    5400 1650
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR014
U 1 1 5BBC0372
P 8300 1000
F 0 "#PWR014" H 8300 850 50  0001 C CNN
F 1 "VDD" H 8317 1173 50  0000 C CNN
F 2 "" H 8300 1000 50  0001 C CNN
F 3 "" H 8300 1000 50  0001 C CNN
	1    8300 1000
	1    0    0    -1  
$EndComp
Connection ~ 7550 1000
Connection ~ 8200 1000
Wire Wire Line
	7550 1000 8200 1000
Wire Wire Line
	6900 1000 7550 1000
Wire Wire Line
	8200 1250 8200 1200
Wire Wire Line
	6900 1250 7550 1250
Wire Wire Line
	7550 1200 7550 1250
Connection ~ 7550 1250
Wire Wire Line
	7550 1250 8200 1250
Wire Wire Line
	7550 1250 7550 1300
Wire Wire Line
	6600 1000 6600 1350
$Comp
L Device:C_Small C11
U 1 1 5BBC3DF2
P 7600 2250
F 0 "C11" V 7371 2250 50  0000 C CNN
F 1 "8.2pF 50V" V 7462 2250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7600 2250 50  0001 C CNN
F 3 "" H 7600 2250 50  0001 C CNN
	1    7600 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	7500 2250 7100 2250
Connection ~ 6600 2250
$Comp
L Device:L L2
U 1 1 5BBC5831
P 8250 2250
F 0 "L2" V 8440 2250 50  0000 C CNN
F 1 "150nH" V 8349 2250 50  0000 C CNN
F 2 "Inductor_SMD:L_0805_2012Metric" H 8250 2250 50  0001 C CNN
F 3 "" H 8250 2250 50  0001 C CNN
	1    8250 2250
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C13
U 1 1 5BBC6203
P 8550 2600
F 0 "C13" H 8642 2646 50  0000 L CNN
F 1 "4.7pF" H 8642 2555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8550 2600 50  0001 C CNN
F 3 "" H 8550 2600 50  0001 C CNN
	1    8550 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 2500 8550 2250
Wire Wire Line
	8550 2250 8400 2250
$Comp
L power:GNDREF #PWR015
U 1 1 5BBC68C9
P 8550 2700
F 0 "#PWR015" H 8550 2450 50  0001 C CNN
F 1 "GNDREF" H 8555 2527 50  0000 C CNN
F 2 "" H 8550 2700 50  0001 C CNN
F 3 "" H 8550 2700 50  0001 C CNN
	1    8550 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:Antenna_Dipole AE1
U 1 1 5BBC6AF7
P 8950 2050
F 0 "AE1" H 9180 1966 50  0000 L CNN
F 1 "ANTENNA" H 9180 1875 50  0000 L CNN
F 2 "!Custom:315MHz Antenna" H 8950 2050 50  0001 C CNN
F 3 "" H 8950 2050 50  0001 C CNN
	1    8950 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 2250 8950 2250
Connection ~ 8550 2250
Wire Wire Line
	9050 2250 9150 2250
Wire Wire Line
	9150 2250 9150 2450
$Comp
L power:GNDREF #PWR016
U 1 1 5BBC796C
P 9150 2450
F 0 "#PWR016" H 9150 2200 50  0001 C CNN
F 1 "GNDREF" H 9155 2277 50  0000 C CNN
F 2 "" H 9150 2450 50  0001 C CNN
F 3 "" H 9150 2450 50  0001 C CNN
	1    9150 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5BBCD24C
P 7100 2600
F 0 "C9" H 7192 2646 50  0000 L CNN
F 1 "2.2pF" H 7192 2555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7100 2600 50  0001 C CNN
F 3 "" H 7100 2600 50  0001 C CNN
	1    7100 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 2250 7100 2500
Connection ~ 7100 2250
Wire Wire Line
	7100 2250 6600 2250
$Comp
L power:GNDREF #PWR012
U 1 1 5BBD22F7
P 7100 2700
F 0 "#PWR012" H 7100 2450 50  0001 C CNN
F 1 "GNDREF" H 7105 2527 50  0000 C CNN
F 2 "" H 7100 2700 50  0001 C CNN
F 3 "" H 7100 2700 50  0001 C CNN
	1    7100 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 2250 8100 2250
$Comp
L MCU_Microchip_ATtiny:ATtiny841-SSU U1
U 1 1 5BBC13E2
P 1800 4300
F 0 "U1" H 1350 4600 50  0000 L CNN
F 1 "ATtiny841-SSU" H 1350 4500 50  0000 L CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 1800 4300 50  0001 C CIN
F 3 "" H 1800 4300 50  0001 L CNN
	1    1800 4300
	1    0    0    -1  
$EndComp
$Comp
L Connector:AVR-ISP-6 J1
U 1 1 5BBDA867
P 950 1400
F 0 "J1" H 950 1900 50  0000 R CNN
F 1 "AVR-ISP-6" H 1300 1800 50  0000 R CNN
F 2 "!Custom:AVR-SMD-Programming-Header" V 700 1450 50  0001 C CNN
F 3 "" H -325 850 50  0001 L CNN
	1    950  1400
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR04
U 1 1 5BBDA98D
P 1800 3150
F 0 "#PWR04" H 1800 3000 50  0001 C CNN
F 1 "VDD" H 1817 3323 50  0000 C CNN
F 2 "" H 1800 3150 50  0001 C CNN
F 3 "" H 1800 3150 50  0001 C CNN
	1    1800 3150
	1    0    0    -1  
$EndComp
Text Label 1350 1200 0    50   ~ 0
MISO
Text Label 1350 1300 0    50   ~ 0
MOSI
Text Label 1350 1400 0    50   ~ 0
SCK
Text Label 1350 1500 0    50   ~ 0
RST
Text Label 2400 4300 0    50   ~ 0
MOSI
Text Label 2400 4200 0    50   ~ 0
MISO
Text Label 2400 4100 0    50   ~ 0
SCK
$Comp
L power:GNDREF #PWR02
U 1 1 5BBDD573
P 850 1800
F 0 "#PWR02" H 850 1550 50  0001 C CNN
F 1 "GNDREF" H 855 1627 50  0000 C CNN
F 2 "" H 850 1800 50  0001 C CNN
F 3 "" H 850 1800 50  0001 C CNN
	1    850  1800
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR05
U 1 1 5BBDD5A8
P 1800 5200
F 0 "#PWR05" H 1800 4950 50  0001 C CNN
F 1 "GNDREF" H 1805 5027 50  0000 C CNN
F 2 "" H 1800 5200 50  0001 C CNN
F 3 "" H 1800 5200 50  0001 C CNN
	1    1800 5200
	1    0    0    -1  
$EndComp
Text Label 2400 4900 0    50   ~ 0
RST
Text Label 2500 3700 0    50   ~ 0
EN
Wire Wire Line
	2500 3700 2400 3700
Text Label 2500 3800 0    50   ~ 0
FSK
Wire Wire Line
	2400 3800 2500 3800
Text Label 2500 3900 0    50   ~ 0
ASK
Wire Wire Line
	2500 3900 2400 3900
$Comp
L Device:Crystal_Small Y1
U 1 1 5BBE5387
P 3100 4700
F 0 "Y1" V 3054 4788 50  0000 L CNN
F 1 "8MHz" V 3145 4788 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_TXC_7A-2Pin_5x3.2mm" H 3100 4700 50  0001 C CNN
F 3 "" H 3100 4700 50  0001 C CNN
	1    3100 4700
	0    1    1    0   
$EndComp
Wire Wire Line
	3100 4600 2400 4600
Wire Wire Line
	2400 4700 2950 4700
Wire Wire Line
	2950 4700 2950 4800
Wire Wire Line
	2950 4800 3100 4800
$Comp
L power:GNDREF #PWR06
U 1 1 5BBE71F3
P 3100 5100
F 0 "#PWR06" H 3100 4850 50  0001 C CNN
F 1 "GNDREF" H 3100 4950 50  0000 C CNN
F 2 "" H 3100 5100 50  0001 C CNN
F 3 "" H 3100 5100 50  0001 C CNN
	1    3100 5100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5BBE843A
P 3100 4950
F 0 "C3" H 3192 4996 50  0000 L CNN
F 1 "22pF" H 3192 4905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3100 4950 50  0001 C CNN
F 3 "" H 3100 4950 50  0001 C CNN
	1    3100 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 4800 3100 4850
Connection ~ 3100 4800
Wire Wire Line
	3100 5050 3100 5100
$Comp
L Device:C_Small C2
U 1 1 5BBEA5F2
P 3100 4500
F 0 "C2" H 3192 4546 50  0000 L CNN
F 1 "22pF" H 3192 4455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3100 4500 50  0001 C CNN
F 3 "" H 3100 4500 50  0001 C CNN
	1    3100 4500
	1    0    0    -1  
$EndComp
Connection ~ 3100 4600
Wire Wire Line
	3100 4400 3500 4400
Wire Wire Line
	3500 4400 3500 5100
Wire Wire Line
	3500 5100 3100 5100
Connection ~ 3100 5100
Wire Wire Line
	1800 3150 1800 3400
$Comp
L Device:C_Small C1
U 1 1 5BBEDA82
P 1150 3250
F 0 "C1" H 1242 3296 50  0000 L CNN
F 1 "0.1uF" H 1242 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1150 3250 50  0001 C CNN
F 3 "" H 1150 3250 50  0001 C CNN
	1    1150 3250
	-1   0    0    1   
$EndComp
$Comp
L power:GNDREF #PWR03
U 1 1 5BBEF097
P 1150 3350
F 0 "#PWR03" H 1150 3100 50  0001 C CNN
F 1 "GNDREF" H 1155 3177 50  0000 C CNN
F 2 "" H 1150 3350 50  0001 C CNN
F 3 "" H 1150 3350 50  0001 C CNN
	1    1150 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 3150 1800 3150
Connection ~ 1800 3150
$Comp
L power:VDD #PWR01
U 1 1 5BBF3443
P 850 900
F 0 "#PWR01" H 850 750 50  0001 C CNN
F 1 "VDD" H 867 1073 50  0000 C CNN
F 2 "" H 850 900 50  0001 C CNN
F 3 "" H 850 900 50  0001 C CNN
	1    850  900 
	1    0    0    -1  
$EndComp
Wire Notes Line
	650  650  650  2050
Wire Notes Line
	650  2050 1550 2050
Wire Notes Line
	1550 2050 1550 650 
Wire Notes Line
	1550 650  650  650 
Text Notes 850  600  0    50   ~ 0
Programming
Wire Wire Line
	6050 1950 6050 2500
Connection ~ 6050 1950
Wire Wire Line
	6050 1950 6200 1950
$Comp
L power:GNDREF #PWR010
U 1 1 5BBFD4F5
P 6050 2700
F 0 "#PWR010" H 6050 2450 50  0001 C CNN
F 1 "GNDREF" H 6055 2527 50  0000 C CNN
F 2 "" H 6050 2700 50  0001 C CNN
F 3 "" H 6050 2700 50  0001 C CNN
	1    6050 2700
	1    0    0    -1  
$EndComp
NoConn ~ 2400 4800
$Comp
L Device:R_Small R2
U 1 1 5BC0B6BF
P 6050 2600
F 0 "R2" H 6109 2646 50  0000 L CNN
F 1 "100k" H 6109 2555 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 6050 2600 50  0001 C CNN
F 3 "" H 6050 2600 50  0001 C CNN
	1    6050 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 4100 2550 4100
Wire Wire Line
	2400 4200 2550 4200
Wire Wire Line
	2400 4300 2550 4300
NoConn ~ 2400 4000
Wire Wire Line
	2400 4900 2550 4900
$Comp
L Device:Battery_Cell BT1
U 1 1 5BBDAE9A
P 2000 1200
F 0 "BT1" H 2118 1296 50  0000 L CNN
F 1 "CR2032" H 2118 1205 50  0000 L CNN
F 2 "Battery:BatteryHolder_Keystone_3034_1x20mm" V 2000 1260 50  0001 C CNN
F 3 "~" V 2000 1260 50  0001 C CNN
	1    2000 1200
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0101
U 1 1 5BBDB4FE
P 2000 1000
F 0 "#PWR0101" H 2000 850 50  0001 C CNN
F 1 "VDD" H 2017 1173 50  0000 C CNN
F 2 "" H 2000 1000 50  0001 C CNN
F 3 "" H 2000 1000 50  0001 C CNN
	1    2000 1000
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0102
U 1 1 5BBDB53B
P 2000 1300
F 0 "#PWR0102" H 2000 1050 50  0001 C CNN
F 1 "GNDREF" H 2005 1127 50  0000 C CNN
F 2 "" H 2000 1300 50  0001 C CNN
F 3 "" H 2000 1300 50  0001 C CNN
	1    2000 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 2250 5000 2250
$Comp
L RF:MICRF112YMM U2
U 1 1 5BC4FF45
P 5400 2350
F 0 "U2" H 5150 3000 50  0000 C CNN
F 1 "MICRF112YMM" H 5700 3000 50  0000 C CNN
F 2 "Package_SO:MSOP-10_3x3mm_P0.5mm" H 5400 1600 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/MICRF112.pdf" H 5100 2400 50  0001 C CNN
	1    5400 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 2250 6600 2250
Wire Wire Line
	5800 1950 6050 1950
Wire Wire Line
	2550 4900 2550 5000
$Comp
L power:VDD #PWR017
U 1 1 5BE8D2F4
P 2800 5000
F 0 "#PWR017" H 2800 4850 50  0001 C CNN
F 1 "VDD" H 2817 5173 50  0000 C CNN
F 2 "" H 2800 5000 50  0001 C CNN
F 3 "" H 2800 5000 50  0001 C CNN
	1    2800 5000
	1    0    0    -1  
$EndComp
Text Notes 3700 2800 0    50   ~ 0
10pF gives\n33.5kHz spread
Wire Notes Line
	2900 4300 2900 5350
Wire Notes Line
	2900 5350 3600 5350
Wire Notes Line
	3600 5350 3600 4300
Wire Notes Line
	3600 4300 2900 4300
Text Notes 2850 5550 0    50   ~ 0
8MHz Internal Oscillator\nmight suffice
Wire Wire Line
	2800 5350 2800 5000
Wire Wire Line
	2550 5350 2800 5350
Wire Wire Line
	2550 5200 2550 5350
$Comp
L Device:R_Small R1
U 1 1 5BE8BD48
P 2550 5100
F 0 "R1" H 2609 5146 50  0000 L CNN
F 1 "100k" H 2609 5055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 2550 5100 50  0001 C CNN
F 3 "" H 2550 5100 50  0001 C CNN
	1    2550 5100
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPST SW1
U 1 1 5BF1F002
P 3250 4150
F 0 "SW1" H 3250 4385 50  0000 C CNN
F 1 "SW_SPST" H 3250 4294 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_PTS645" H 3250 4150 50  0001 C CNN
F 3 "" H 3250 4150 50  0001 C CNN
	1    3250 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 4400 2850 4400
Wire Wire Line
	2850 4400 2850 4150
Wire Wire Line
	2850 4150 3050 4150
Wire Wire Line
	3450 4150 3750 4150
Wire Wire Line
	3750 4150 3750 4450
$Comp
L power:GNDREF #PWR018
U 1 1 5BF26970
P 3750 4450
F 0 "#PWR018" H 3750 4200 50  0001 C CNN
F 1 "GNDREF" H 3755 4277 50  0000 C CNN
F 2 "" H 3750 4450 50  0001 C CNN
F 3 "" H 3750 4450 50  0001 C CNN
	1    3750 4450
	1    0    0    -1  
$EndComp
$EndSCHEMATC
