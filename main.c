#include "uart.h"
#include "stdlib.h"
#include <stdlib.h>
#include "config.h"
#include "gpio.h"
#include <string.h>
#include "interrupt.h"
#include "ds18b20.h"
#include "adc.h"
#include <stdbool.h>
#include "i2c.h"
#include "i2c_api.h"
#include <math.h>
//#include "soft_uart.h"
#define TRUE 1
#define FALSE 0
#define limit 4
#define BUFFER_SIZE 1200
// Wire library - used for I2C communication
int ADXL345 = 0x53; // The ADXL345 sensor I2C address
float X_out, Y_out, Z_out,prev_x,prev_y,prev_z;  // Outputs
signed short sX_out, sY_out, sZ_out;  // Outputs
char TXBuffer[BUFFER_SIZE];
char location[] = "{\"value\":"; //TODO
char data[] = "{\"value\": ";
char payload[100] = { 0,};
char truck_id[] = "KA-03-MW-123";
int response_ok = 0;
int connection_id = 0;
int pressed;
int temp;
int para_ok;
float celcius, sign;
US adc_data = 0;
float volt_per_division = 0.0009;
float fuel_level = 0.0, initial_fuel, fuel_diff;
int low_fuel_alert;
bool fuel_theft_initial_save = true;
bool accident_alert_sent = true;
bool fuel_theft_alert = true;
int http_init = 0;
char GPRMC_data[300] = { 0, };
char * values[300];
//char GPRMC_data[300]="GPRMC,061655.00,A,0830.56694,N,07657.71509,E,2.821,,090920,,,A*70";
char * utc_time;
char * navig_status;
char * lat;
char * lat_direction;
char * lon;
char * lon_direction;
char * sog;
char * azimuth;
char * utc_date;
char * utc_date;
double latitude,longitude;
float speed;
char str1[100] = { 0, };
void dateParse(char * date_str) {
	printf("DATE \t\t: %c%c/%c%c/%c%c\n", date_str[0], date_str[1], date_str[2], date_str[3], date_str[4], date_str[5]);
}
void timeParse(char * time_str) {
	printf("UTC TIME \t: %c%c:%c%c:%c%c\n", time_str[0], time_str[1], time_str[2], time_str[3], time_str[4], time_str[5]);
}
/*GPRMC,061655.00,A,0830.56694,N,07657.71509,E,2.821,,090920,,,A*70*/
/*--------------------------------------------------------------------------\
 |            | NMEA          | Decimal                                     ||
 |------------|---------------|-----------------------------------------------
 | latitude   | 2511.120738   | 25 + (11.120738/60) = 25.185345633          ||
 |------------|---------------|-----------------------------------------------
 | longitude  | 05516.820909  | 055 + (16.820909/60) = 55.280348483         ||
 \--------------------------------------------------------------------------*/
#define MAX_LONGITUDE    180
#define MAX_LATITUDE     90
#define LAT 1
#define LON 2
/**
 * Convert latitude,longitude from nmea to decimal
 * @param type lat = 1, lon = 2
 */
double degToDecimal(char *nmea, char type, unsigned char *dir) {
	int idx, dot = 0;
	double dec = 0;
	for (idx = 0; idx < strlen(nmea); idx++) {
		if (nmea[idx] == '.') {
			dot = idx;
			break;
		}
	}
	if (dot < 3)
		return 0;
	int i, dd;
	double mm;
	char cdd[5], cmm[10];
	memset(&cdd, 0, 5);
	memset(&cmm, 0, 10);
	strncpy(cdd, nmea, dot - 2);
	strcpy(cmm, nmea + dot - 2);
	dd = atoi(cdd);
	mm = atof(cmm);
	dec = dd + (mm / 60);
	if (type == 1 && dec > MAX_LATITUDE)
		return 0;
	else if (type == 2 && dec > MAX_LONGITUDE)
		return 0;
	if (strcmp(dir, "N") == 0 || strcmp(dir, "E") == 0)
		return dec;
	else
		return -1 * dec;
}
/**
 * Splits a string by comma.
 *
 * string is the string to split, will be manipulated. Needs to be
 *        null-terminated.
 * values is a char pointer array that will be filled with pointers to the
 *        splitted values in the string.
 * max_values is the maximum number of values to be parsed.
 *
 * Returns the number of values found in string.
 */
static int split_string_by_comma(char *string, char **values, int max_values)
{
	int i = 0;
	values[i++] = string;
	while (i < max_values && NULL != (string = strchr(string, ','))) {
		*string = '\0';
		values[i++] = ++string;
	}
	return i;
}
void getGPSData() {
	char error;
	char data;
	int i = 0;
	char *token;
	const char s1[2] = ",";
	while (1) {
		memset(GPRMC_data, 0, i);
		data = uart_getchar(UART_2, &error);
		//printf("%c", data);
		if (data == '$') {
			data = uart_getchar(UART_2, &error);
			if (data == 'G') {
				data = uart_getchar(UART_2, &error);
				if (data == 'P') {
					data = uart_getchar(UART_2, &error);
					if (data == 'R') {
						GPRMC_data[i] = data;
						while (data != '\n') {
							i++;
							data = uart_getchar(UART_2, &error);
							GPRMC_data[i] = data;
						}
						i++;
						GPRMC_data[i] = '\0';
						//printf("%s\n", GPRMC_data);
						split_string_by_comma(GPRMC_data,values,10);
						/*for (int k = 0; k < 10; k++)
							printf("%s\n", values[k]);*/
						i = 0;
						break;
				   }
				}
			}
		}
	}
}
int send_string(char *str)
{
    char error;
    response_ok = 0;
    while (*str != '\0')
    {
        uart_putchar(UART_1, *str, &error);
        str++;
    }
    return 0;
}
int send_char(char code)
{
    char error;
    uart_putchar(UART_1, code, &error);
    return 0;
}
void receive_string(char *str)
{
    char error;
    char data;
    while (1)
    {
        data = uart_getchar(UART_1, &error);
        *str = data;
        str++;
        if (data == 'O')
        {
            data = uart_getchar(UART_1, &error);
            *str = data;
            str++;
            if (data == 'K')
            {
                response_ok = 1;
                break;
            }
        }
        if (data == 'E')
        {
            data = uart_getchar(UART_1, &error);
            *str = data;
            str++;
            if (data == 'R')
            {
                data = uart_getchar(UART_1, &error);
                *str = data;
                str++;
                if (data == 'R')
                {
                    response_ok = 0;
                    break;
                }
            }
        }
        if (data == 'D')
        {
            data = uart_getchar(UART_1, &error);
            *str = data;
            str++;
            if (data == 'O')
            {
                data = uart_getchar(UART_1, &error);
                *str = data;
                str++;
                if (data == 'W')
                {
                    response_ok = 1;
                    break;
                }
            }
        }
        if (data == '>')
        {
            response_ok = 0;
            break;
        }
    }
}
int gsm_send(char *sstr)
{
    char response[100] = {
        0,
    };
    send_string(sstr);
    receive_string(response);
    printf("%s\n", response);
    if (response_ok)
        return 1;
    else
        return 0;
}
void AT()
{
    char str[100] = {
        0,
    };
    send_string("AT\r\n");
    receive_string(str);
    //printf("STR: %s\n",str);
    if (response_ok)
        printf("AT DONE\n");
    else
        printf("error in AT operation\n");
}
void AT_CREG()
{
    char str[100] = {
        0,
    };
    send_string("AT+CREG?\r\n");
    receive_string(str);
    if (response_ok)
        printf("AT DONE\n");
    else
        printf("error in AT operation\n");
}
void AT_SAPBR1()
{
    char str[100] = {
        0,
    };
    send_string("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
    receive_string(str);
    if (response_ok)
        printf("AT SAPBR1 DONE\n");
    else
        printf("error in SAPBR1 operation\n");
}
void AT_SAPBR2()
{
    char str[100] = {
        0,
    };
    send_string("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
    receive_string(str);
    if (response_ok)
        printf("AT SAPBR2 DONE\n");
    else
        printf("error in SAPBR2 operation\n");
}
void AT_SAPBR3()
{
    char str[100] = {
        0,
    };
    send_string("AT+SAPBR=1,1\r\n");
    receive_string(str);
    if (response_ok)
        printf("AT SAPBR3 DONE\n");
    else
        printf("error in SAPBR3 operation\n");
}
void AT_SAPBR4()
{
    char str[100] = {
        0,
    };
    send_string("AT+SAPBR=2,1\r\n");
    receive_string(str);
    if (response_ok)
        printf("AT SAPBR4 DONE\n");
    else
        printf("error in SAPBR4 operation\n");
}
void AT_HTTPINIT()
{
    char str[100] = {
        0,
    };
    send_string("AT+HTTPINIT\r\n");
    receive_string(str);
    if (response_ok)
    {
        printf("AT HTTP INIT DONE\n");
        http_init = 1;
    }
    else
    {
        printf("error in HTTP INIT operation\n");
        http_init = 0;
    }
}
void AT_HTTPPARA1()
{
    char str[100] = {
        0,
    };
    send_string("AT+HTTPPARA=\"CID\",1\r\n");
    receive_string(str);
    if (response_ok){
        printf("AT HTTP PARA 1 DONE\n");
        para_ok=1;
        }
    else{
        para_ok=0;
        printf("error in HTTP PARA1 operation\n");
        }
}
void AT_HTTPPARA2()
{
    char str[150] = {
        0,
    };
    send_string("AT+HTTPPARA=\"URL\",\"http://industrial.api.ubidots.com/api/v1.6/devices/testing/demoo/values?token=BBFF-zhcPS3n6jv6JRwD2GFCpaRIqo07nVb\"\r\n");
    receive_string(str);
    if (response_ok)
        printf("AT HTTP PARA 2 DONE\n");
    else
        printf("error in HTTP PARA 2 operation\n");
}
void AT_HTTPPARA2_SOS()
{
    char str[150] = {
        0,
    };
    int alert = 1;
    send_string("AT+HTTPPARA=\"URL\",\"http://industrial.api.ubidots.com/api/v1.6/devices/testing/sos/values?token=BBFF-zhcPS3n6jv6JRwD2GFCpaRIqo07nVb\"\r\n");
    sprintf(payload, "%s%d}\r\n", data, alert);
    receive_string(str) ;
    if (response_ok)
        printf("AT HTTP PARA 2 DONE\n");
    else printf("error in HTTP PARA 2 operation\n");
}
void AT_HTTPPARA2_temp()
{
    char str[150] = {
        0,
    };
    send_string("AT+HTTPPARA=\"URL\",\"http://industrial.api.ubidots.com/api/v1.6/devices/testing/temp/values?token=BBFF-zhcPS3n6jv6JRwD2GFCpaRIqo07nVb\"\r\n");
    sprintf(payload, "%s%d}\r\n", data, (int)celcius);
    receive_string(str);
    if (response_ok)
        printf("AT HTTP PARA 2 DONE\n");
    else
        printf("error in HTTP PARA 2 operation\n");
}
void AT_HTTPPARA2_speed_gps()
{
    char str[150] = {
        0,
    };
    send_string("AT+HTTPPARA=\"URL\",\"http://industrial.api.ubidots.com/api/v1.6/devices/testing/speed/values?token=BBFF-zhcPS3n6jv6JRwD2GFCpaRIqo07nVb\"\r\n");
    //27, \"context\":{\"lat\":37.773, \"lng\":-122.431}}"
    sprintf(payload, "%s%f, \"context\":{\"lat\":%0.3f, \"lng\":%0.3f}}\r\n", data,speed,latitude,longitude);
    receive_string(str);
    printf(payload);
    if (response_ok)
        printf("AT HTTP PARA 2 DONE\n");
    else
        printf("error in HTTP PARA 2 operation\n");
}
void AT_HTTPPARA2_fuel()
{
    char str[150] = {
        0,
    };
    send_string("AT+HTTPPARA=\"URL\",\"http://industrial.api.ubidots.com/api/v1.6/devices/testing/fuel/values?token=BBFF-zhcPS3n6jv6JRwD2GFCpaRIqo07nVb\"\r\n");
    sprintf(payload, "%s%0.2f}\r\n", data, fuel_level);
    receive_string(str);
    if (response_ok)
        printf("AT HTTP PARA 2 DONE\n");
    else
        printf("error in HTTP PARA 2 operation\n");
}
void AT_HTTPPARA2_fuel_theft()
{
    char str[150] = {
        0,
    };
    int alert = 1;
    send_string("AT+HTTPPARA=\"URL\",\"http://industrial.api.ubidots.com/api/v1.6/devices/testing/fuel_theft/values?token=BBFF-zhcPS3n6jv6JRwD2GFCpaRIqo07nVb\"\r\n");
    sprintf(payload, "%s%d}\r\n", data, alert);
    receive_string(str);
    if (response_ok)
        printf("AT HTTP PARA 2 DONE\n");
    else
        printf("error in HTTP PARA 2 operation\n");
}
void AT_HTTPPARA2_accident()
{
char str[150] = {
        0,
    };
    int alert = 1;
    send_string("AT+HTTPPARA=\"URL\",\"http://industrial.api.ubidots.com/api/v1.6/devices/testing/accident/values?token=BBBFF-zhcPS3n6jv6JRwD2GFCpaRIqo07nVb\"\r\n");
    sprintf(payload, "%s%d}\r\n", data, alert);
    receive_string(str);
    if (response_ok)
        printf("AT HTTP PARA 2 DONE\n");
    else
        printf("error in HTTP PARA 2 operation\n");
}
void AT_HTTPPARA3()
{
    char str[100] = {
        0,
    };
    send_string("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
    receive_string(str);
    if (response_ok)
        printf("AT HTTP PARA 3 DONE\n");
    else
        printf("error in HTTP PARA3 operation\n");
}
void AT_HTTPDATA()
{
    if (http_init && para_ok)
    {
        char response[100] = {
            0,
        };
        char send_str[100] = {
            0,
        };
        int len = strlen(payload) + 1;
        sprintf(send_str, "AT+HTTPDATA=%d,10000\r\n", len);
        send_string(send_str);
        receive_string(response);
        if (response_ok)
            printf("AT HTTPDATA DONE\n");
        else
            printf("AT HTTPDATA error\n");
        udelay(100000);
        send_string(payload);
        receive_string(response);
        if (response_ok)
            printf("payload DONE\n");
        else
            printf("payload error\n");
    }
}
void AT_HTTPACTION()
{
    char str[100] = {
        0,
    };
    send_string("AT+HTTPACTION=1\r\n");
    receive_string(str);
    if (response_ok)
        printf("AT HTTP action DONE\n");
    else
        printf("error in HTTP action operation\n");
}
void AT_HTTPREAD()
{
    char str[100] = {
        0,
    };
    send_string("AT+HTTPREAD\r\n");
    receive_string(str);
    printf("STR: %s\n", str);
    if (response_ok)
        printf("AT HTTP read DONE\n");
    else
        printf("error in HTTP read operation\n");
}
void AT_HTTPTERM()
{
    char str[100] = {
        0,
    };
    send_string("AT+HTTPTERM\r\n");
    receive_string(str);
    if (response_ok)
        printf("AT HTTP term DONE\n");
    else
        printf("error in HTTP term operation\n");
}
void low_fuel_alert_trigger()
{
    printf("entering low fuel function \n");
    char fuel_location[] = "Low fuel alert refuel at https://www.google.com/maps/search/?api=1&query=gas+station\r\n";
    gsm_send("AT+CIMI\r\n"); //IMEI display
    udelay(1000);
    gsm_send("AT+CMGF=1\r\n"); //SMS mode
    udelay(1000);
    gsm_send("AT+CSCS=\"GSM\"\r\n"); //Type GSM
    udelay(1000);
    gsm_send("AT+CMGS=\"+919845142690\"\r\n"); //Set mobile number -replace it with a vaild mobile number
    udelay(10000);
    gsm_send(fuel_location); //SMS content
    udelay(1000);
    send_char(0x1a); //Ctrl+Z
    printf("Message sent");
    udelay(1000);
    low_fuel_alert = 0;
    udelay(500000);
}
void gpio_intr_handler()
{
    char sos_location[100] = "SOS pressed at http://maps.google.co.uk/maps?f=q&hl=en&q=";
    float latitude1 = 25.324, longitude1 = 23.2343; //to be removed only for testing TODO
    sprintf(sos_location, "%s%f,%f\r\n", sos_location, latitude, longitude);
    if (pressed)
    {
        pressed = 0;
        printf("\n\n BUTTON IS PRESSED \n\n");
        udelay(200000);
        //sending SMS
        udelay(2000);
        gsm_send("AT+CIMI\r\n"); //IMEI display
        udelay(2000);
        gsm_send("AT+CMGF=1\r\n"); //SMS mode
        udelay(1000);
        gsm_send("AT+CMGS=\"+919845142690\"\r\n"); //Set mobile number -replace it with a vaild mobile number
        udelay(10000);
        gsm_send(sos_location); //SMS content
        udelay(1000);
        send_char(0x1a); //Ctrl+Z
        printf("Message sent\n");
        udelay(1000);
        printf("Exiting SOS\n");
        udelay(200000);
        pressed = 1;
    }
    return;
}
void fuel_theft_send_alert()
{
    char fuel_theft_sms[100] = "Fuel theft detected on truck";
    sprintf(fuel_theft_sms, "%s%s\r\n", fuel_theft_sms, truck_id);
    udelay(1000);
    /*AT_HTTPINIT();
    udelay(2000);
    AT_HTTPPARA1();
    udelay(2000);
    AT_HTTPPARA2_fuel_theft();
    udelay(2000);
    AT_HTTPPARA3();
    udelay(2000);
    AT_HTTPDATA();
    udelay(2000);
    AT_HTTPACTION();
    udelay(1000000);
    AT_HTTPTERM();
    udelay(1000);*/
    gsm_send("AT+CIMI\r\n"); //IMEI display
    udelay(1000);
    gsm_send("AT+CMGF=1\r\n"); //SMS mode
    udelay(1000);
    gsm_send("AT+CSCS=\"GSM\"\r\n"); //Type GSM
    udelay(1000);
    gsm_send("AT+CMGS=\"+919845142690\"\r\n"); //Set mobile number -replace it with a vaild mobile number
    udelay(10000);
    gsm_send(fuel_theft_sms); //SMS content
    udelay(1000);
    send_char(0x1a); //Ctrl+Z
    printf("Message sent");
    udelay(1000);
}
void http_commands()
{
    AT_HTTPPARA3();
    udelay(2000);
    AT_HTTPDATA();
    udelay(2000);
    AT_HTTPACTION();
    udelay(200000);
    AT_HTTPTERM();
    udelay(1000);
}
void accelorometer_init(){
i2c_init(I2C_1); //System clock =40MHz and I2C clock =100 kHz
	i2c_beginTransmission(ADXL345); // Start communicating with the device
	i2c_write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
	// Enable measurement
	i2c_write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable
	i2c_endTransmission(TRUE);
	udelay(100);
	i2c_beginTransmission(ADXL345);
	i2c_write(0x31);  // Range set to 16g/ Full resolution
	i2c_write(0xb);
	i2c_endTransmission(TRUE);
	udelay(100);
	//X-axis
	i2c_beginTransmission(ADXL345);
	i2c_write(0x1E);  // X-axis offset register
	i2c_write(97);
	i2c_endTransmission(TRUE);
	udelay(100);
	//Y-axis
	i2c_beginTransmission(ADXL345);
	i2c_write(0x1F);  // Y-axis offset register
	i2c_write(255);
	i2c_endTransmission(TRUE);
	udelay(100);
	//Z-axis
	i2c_beginTransmission(ADXL345);
	i2c_write(0x20);  // Z-axis offset register
	i2c_write(-9);
	i2c_endTransmission(TRUE);
	udelay(100);
}
void send_accident_alert(){
int alert=1;
 char accident_location[100] = "Accident at http://maps.google.co.uk/maps?f=q&hl=en&q=";
    float latitude1 = 25.324, longitude1 = 23.2343; //to be removed only for testing TODO
    sprintf(accident_location, "%s%f,%f\r\n", accident_location, latitude, longitude);
/*AT_HTTPINIT();
        udelay(2000);
        AT_HTTPPARA1();
        udelay(2000);
        AT_HTTPPARA2_accident();
        udelay(2000);
        http_commands();*/
        gsm_send("AT+CIMI\r\n"); //IMEI display
    udelay(1000);
    gsm_send("AT+CMGF=1\r\n"); //SMS mode
    udelay(1000);
    gsm_send("AT+CSCS=\"GSM\"\r\n"); //Type GSM
    udelay(1000);
    gsm_send("AT+CMGS=\"+919845142690\"\r\n"); //Set mobile number -replace it with a vaild mobile number
    udelay(10000);
    gsm_send(accident_location); //SMS content
    udelay(1000);
    send_char(0x1a); //Ctrl+Z
    printf("Message sent");
    udelay(1000);
}
void check_accident(){
i2c_beginTransmission(ADXL345);
		i2c_write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
		i2c_endTransmission(TRUE);
		i2c_requestFrom(ADXL345, 6, TRUE); // Read 6 registers total, each axis value is stored in 2 registers
		sX_out = (i2c_read() | (i2c_read() << 8)); // X-axis value
		sY_out = (i2c_read() | (i2c_read() << 8)); // Y-axis value
		sZ_out = (i2c_read() | (i2c_read() << 8)); // Z-axis value
		//printf("\rXa= %d,  Ya= %d, Za= %d",sX_out,sY_out,sZ_out);
		X_out = ((float) sX_out) / 64; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
		Y_out = ((float) sY_out) / 64;
		Z_out = ((float) sZ_out) / 64;
		prev_x = fabs(X_out)-fabs(((float) sX_out) / 64);
               prev_y = fabs(Y_out)-fabs(((float) sY_out) / 64);
               prev_z = fabs(Z_out)-fabs(((float) sY_out) / 64);
               if(prev_x >limit|prev_y>limit|prev_z>limit){
               printf("first check done\n");}
                //getGPSData();
            //float speed_check = atof(values[7]); //uncomment once gps is working
            int speed_check1=4; //TODO
          
                if(X_out>8 && accident_alert_sent && speed_check1<5){
             printf("High probability a Accident has taken place\n");
               send_accident_alert();
               accident_alert_sent=false;
            }
            
               else printf("SAFE\n");
//printf("\rXa= %5.2f,  Ya= %5.2f, Za= %5.2f", X_out, Y_out, Z_out);
}
/**
  @fn main
  @brief transmit and reception through uart
  @details 1 character is transmitted and received through uart
  @param[in] No input parameter.
  @param[Out] No ouput parameter.
  @return Void function.
*/
void main()
{

	
	
	//**************************************************************************************************
    irq_register_handler(GPIO_0_IRQ, gpio_intr_handler);
    interrupt_enable(GPIO_0_IRQ);
    pressed =1; 
    accelorometer_init();
    int wait=1;
	char error;
	char str[100] = { 0, };
	unsigned char data = 'A';
	printf("\n\r *****************************************************************************");
	printf("\n\r INFO: Connect GPS6MV2 module to UART 1 ");		
	printf("\n\r *****************************************************************************");
    low_fuel_alert = 1;
    printf("\n\r ****\n\r");
    printf("\n\r INFO: Connect GSM module [SIM800A] to UART 1 ");
    printf("\n\r ****");
    uart_set_baud_rate(UART_1, 9600, 40000000);
    udelay(1000);
    gsm_send("AT\n");
    udelay(1000);
    AT_CREG();
    udelay(2000);
    AT_SAPBR1(); //Set parameters, set identifier
    udelay(2000);
    AT_SAPBR2();
    udelay(2000);
    AT_SAPBR3();
    udelay(2000);
    AT_SAPBR4();
    udelay(2000);
uart_set_baud_rate(UART_2, 9600, 40000000);
    while (1)
    {
    check_accident();
		getGPSData();
		/*for (int k = 0; k < 10; k++)
			printf(">> %s\n", values[k]);*/
		printf("\n=============================================\n");
		if(strlen(values[1])>5)
			timeParse(values[1]);
		//printf("utc_date \t: %s\n",utc_date);
		if(strlen(values[1])>5)
			dateParse(values[9]);
		if (strcmp(values[2], "A") == 0) {
			//printf("utc_time \t: %s\n",utc_time);
			//printf("getGPSData over\n");
			//printf("navig_status \t: %s\n",navig_status);
			//printf("latitude \t: %s\n",lat);
			//printf("lat_direction \t: %s\n",lat_direction);
			latitude = degToDecimal(values[3], LAT, values[4]);
			printf("latitude \t: %f\n", latitude);
			//printf("longitude \t: %s\n",lon);
			//printf("lon_direction \t: %s\n",lon_direction);
			longitude = degToDecimal(values[5], LON, values[6]);
			printf("longitude \t: %f\n", longitude);
			printf("speed over ground \t: %s\n",values[7]);
			speed=atof(values[7]);
			AT_HTTPINIT();
        udelay(2000);
        AT_HTTPPARA1();
        udelay(2000);
        AT_HTTPPARA2_speed_gps();
        udelay(2000);
        http_commands();
			} else {
			printf("Waiting for Valid GPS data...\n");
		}
		printf("\n=============================================\n");
        /*------------------------------------------------------------------
                                 TEMPERATURE SENSOR
      -------------------------------------------------------------------*/
      check_accident();
        temp = getRawTemperature();
        sign = 1.0;
        if (temp & TEMP_SIGN_BIT)
        {
            sign = -1.0;
            temp = (temp & (~TEMP_SIGN_BIT));
        }
        celcius = sign * ((float)temp / 16.0);
        printf("Calculated Temp : %f Celcius\n", celcius);
        for (int i = 0; i < 2060000; i++) // 1 sec delay
            delay_1_us();
            check_accident();
        udelay(1000);
        AT_HTTPINIT();
        udelay(2000);
        AT_HTTPPARA1();
        udelay(2000);
        AT_HTTPPARA2_temp();
        udelay(2000);
        http_commands();
        /*-----------------------------------------------------------------------------------
                                            FUEL SENSOR
      -------------------------------------------------------------------------------------*/
        check_accident();
        adc_data = adc_analogRead(A1);
        printf("Checking fuel status \n");
        fuel_level = (adc_data * volt_per_division);
        printf("\r fuel data: %0.2f V\n", fuel_level);
        check_accident();
        //sending fuel data to cloud
        AT_HTTPINIT();
        udelay(2000);
        AT_HTTPPARA1();
        udelay(2000);
        AT_HTTPPARA2_fuel();
        udelay(2000);
        http_commands();
        check_accident();
        //low fuel alert
        if (fuel_level < 1.2)
        {
            if (low_fuel_alert)
            {
                printf("sending low fuel alert\n");
                low_fuel_alert_trigger();
            }
        }
        if (fuel_level > 2.5)
        {
            low_fuel_alert = 1;
        }
        check_accident();
        //fuel theft alert check
        udelay(10000);
        float speed1 = 0.4; //TODO
        if ((speed1 < 0.5) && (fuel_theft_initial_save == true))
        {
            printf("entering save\n");
            initial_fuel = fuel_level;
            fuel_theft_initial_save = false;
            printf("saved value = %f\n", initial_fuel);
        }
        check_accident();
        fuel_diff = initial_fuel - fuel_level;
        printf("fuel diff= %f\n", fuel_diff);
        if (fuel_diff > 0.5 && fuel_theft_alert == true)
        {
            printf("STOLEN\n");
            fuel_theft_send_alert();
            fuel_theft_alert = false;
        }
        if (speed1 > 5)
        { //TODO
            fuel_theft_alert = true;
            fuel_theft_initial_save = true;
            accident_alert_sent=true;
        }
        check_accident();
    }
    }
