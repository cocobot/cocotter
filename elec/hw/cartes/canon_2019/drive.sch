EESchema Schematic File Version 4
LIBS:canon_2019-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 6
Title "Canon"
Date ""
Rev "2019"
Comp "Cocobot"
Comment1 "Version originale: Brushless SSLv2 - R. Deniéport"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 3500 2200 6750 950 
U 5C9125DE
F0 "U MTR PHASE" 50
F1 "phase.sch" 50
F2 "Vdrv" I L 3500 2300 50 
F3 "PWM_3v3" I L 3500 2650 50 
F4 "EN_3v3" I L 3500 2750 50 
F5 "Vpwr" I L 3500 2400 50 
F6 "PH" O R 10250 2350 50 
F7 "I_0-3v3" O R 10250 2450 50 
F8 "REF_1v65" I L 3500 2500 50 
$EndSheet
$Sheet
S 3500 3500 6750 950 
U 5C89589D
F0 "V MTR PHASE" 50
F1 "phase.sch" 50
F2 "Vdrv" I L 3500 3600 50 
F3 "PWM_3v3" I L 3500 3950 50 
F4 "EN_3v3" I L 3500 4050 50 
F5 "Vpwr" I L 3500 3700 50 
F6 "PH" O R 10250 3650 50 
F7 "I_0-3v3" O R 10250 3750 50 
F8 "REF_1v65" I L 3500 3800 50 
$EndSheet
$Sheet
S 3500 4750 6750 950 
U 5C895A12
F0 "W MTR PHASE" 50
F1 "phase.sch" 50
F2 "Vdrv" I L 3500 4850 50 
F3 "PWM_3v3" I L 3500 5200 50 
F4 "EN_3v3" I L 3500 5300 50 
F5 "Vpwr" I L 3500 4950 50 
F6 "PH" O R 10250 4900 50 
F7 "I_0-3v3" O R 10250 5000 50 
F8 "REF_1v65" I L 3500 5050 50 
$EndSheet
Text HLabel 3150 1800 1    50   Input ~ 0
Vdrv
Wire Wire Line
	3150 1800 3150 2300
Wire Wire Line
	3150 2300 3500 2300
Wire Wire Line
	3150 2300 3150 3600
Wire Wire Line
	3150 3600 3500 3600
Connection ~ 3150 2300
Wire Wire Line
	3150 3600 3150 4850
Wire Wire Line
	3150 4850 3500 4850
Connection ~ 3150 3600
Text HLabel 3000 1700 1    50   Input ~ 0
Vpwr
Wire Wire Line
	3000 1700 3000 2400
Wire Wire Line
	3000 2400 3500 2400
Wire Wire Line
	3000 2400 3000 3700
Wire Wire Line
	3000 3700 3500 3700
Connection ~ 3000 2400
Wire Wire Line
	3000 3700 3000 4950
Wire Wire Line
	3000 4950 3500 4950
Connection ~ 3000 3700
Text HLabel 2450 2650 0    50   Input ~ 0
U_PWM_3v3
Wire Wire Line
	2450 2650 3500 2650
Text HLabel 2450 2750 0    50   Input ~ 0
U_EN_3v3
Wire Wire Line
	2450 2750 3500 2750
Text HLabel 2450 3950 0    50   Input ~ 0
V_PWM_3v3
Wire Wire Line
	2450 3950 3500 3950
Text HLabel 2450 4050 0    50   Input ~ 0
V_EN_3v3
Wire Wire Line
	2450 4050 3500 4050
Text HLabel 2450 5200 0    50   Input ~ 0
W_PWM_3v3
Wire Wire Line
	2450 5200 3500 5200
Text HLabel 2450 5300 0    50   Input ~ 0
W_EN_3v3
Wire Wire Line
	2450 5300 3500 5300
Text HLabel 2850 2000 1    50   Input ~ 0
REF_1v65
Wire Wire Line
	2850 2000 2850 2500
Wire Wire Line
	2850 2500 3500 2500
Wire Wire Line
	2850 2500 2850 3800
Wire Wire Line
	2850 3800 3500 3800
Connection ~ 2850 2500
Wire Wire Line
	2850 3800 2850 5050
Wire Wire Line
	2850 5050 3500 5050
Connection ~ 2850 3800
Text HLabel 10600 2350 2    50   Output ~ 0
U_PH
Wire Wire Line
	10600 2350 10250 2350
Text HLabel 10600 2450 2    50   Output ~ 0
I_U_0-3v3
Wire Wire Line
	10600 2450 10250 2450
Text HLabel 10600 3650 2    50   Output ~ 0
V_PH
Wire Wire Line
	10600 3650 10250 3650
Text HLabel 10600 3750 2    50   Output ~ 0
I_V_0-3v3
Wire Wire Line
	10600 3750 10250 3750
Text HLabel 10600 4900 2    50   Output ~ 0
W_PH
Wire Wire Line
	10600 4900 10250 4900
Text HLabel 10600 5000 2    50   Output ~ 0
I_W_0-3v3
Wire Wire Line
	10600 5000 10250 5000
Text HLabel 950  950  1    50   Input ~ 0
Vpwr
$Comp
L Device:CP C14
U 1 1 5C898D04
P 950 1300
F 0 "C14" H 1068 1346 50  0000 L CNN
F 1 "CP" H 1068 1255 50  0000 L CNN
F 2 "espitall:c_elec_6.3x7.7" H 988 1150 50  0001 C CNN
F 3 "~" H 950 1300 50  0001 C CNN
	1    950  1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  1150 950  950 
$Comp
L power:GND #PWR0136
U 1 1 5C8991D5
P 950 1700
F 0 "#PWR0136" H 950 1450 50  0001 C CNN
F 1 "GND" H 955 1527 50  0000 C CNN
F 2 "" H 950 1700 50  0001 C CNN
F 3 "" H 950 1700 50  0001 C CNN
	1    950  1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  1700 950  1450
Text HLabel 1350 950  1    50   Input ~ 0
Vpwr
Wire Wire Line
	1350 1150 1350 950 
$Comp
L power:GND #PWR0137
U 1 1 5C899CE4
P 1350 1700
F 0 "#PWR0137" H 1350 1450 50  0001 C CNN
F 1 "GND" H 1355 1527 50  0000 C CNN
F 2 "" H 1350 1700 50  0001 C CNN
F 3 "" H 1350 1700 50  0001 C CNN
	1    1350 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 1700 1350 1450
$Comp
L Device:C C15
U 1 1 5C89A213
P 1350 1300
F 0 "C15" H 1465 1346 50  0000 L CNN
F 1 "2u2" H 1465 1255 50  0000 L CNN
F 2 "espitall:C_0603" H 1388 1150 50  0001 C CNN
F 3 "~" H 1350 1300 50  0001 C CNN
	1    1350 1300
	1    0    0    -1  
$EndComp
$EndSCHEMATC
