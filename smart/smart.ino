#define averaging 128
#define  shift  7


#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "LedControl.h"
LedControl lc=LedControl(25,22,23,6);

int i_led;

int displays[12][3]={
{0, 1, 2},
{8, 9,10},
{16,17,18},
{3, 4, 5},
{11,12,13},
{19,20,21},
{6, 7,32}, 
{14,15,28},
{22,23,24},
{33,34,35},
{29,30,31},
{25,26,27}};

int st_leds[12][2]={
  {0,  7},
  {8, 15},
  {16,23},
  {6,  5},
  {14,13},
  {22,21},
  {4,  3},
  {12,11},
  {20,19},
  {2,  1},
  {10, 9},
  {18,17}
};

const int oneWirePinsCount=3;
const int tsmCount=4;
const int pressureCount=4;
const float tks=0.00428;
float TSM_ratios[4]={1, 1, 1, 1};
float temps[]={0,0,0,0,0};
float err_temps[]={0,0,0,0,0};
int max_temps[]={50,50,50,50,50};
float pressures[]={0,0,0,0,0};
float err_pressures[]={0,0,0,0,0};
int max_pressures[]={512,512,512,512,512};
int leds[]={1,1,1,1,1,1,1,1,1,1,1,1};
int temps_map[]={11, 8, 10, 4, 9};
int press_map[]={5, 7, 1, 6, 2};
const int conf_m_pin=17;
const int conf_1_pin=16;
const int conf_2_pin=18;
boolean temp_source=true;
boolean is_ok=false;

int analog_data[16][averaging+1];

int TSMs[4][3]={{10, 9, 8},{4, 7, 6},{11,2,3},{5,1,0}};
OneWire ds18x20[]={OneWire(21),OneWire(20), OneWire(19)};
DallasTemperature sensor[oneWirePinsCount];

unsigned long counter=0;
unsigned long measure_n=0;

// чтение
float EEPROM_float_read(int addr) {    
  byte raw[4];
  for(byte i = 0; i < 4; i++) raw[i] = EEPROM.read(addr+i);
  float &num = (float&)raw;
  return num;
}

// запись
void EEPROM_float_write(int addr, float num) {
  byte raw[4];
  (float&)raw = num;
  for(byte i = 0; i < 4; i++) EEPROM.write(addr+i, raw[i]);
}

void readAnalogCount(int c){
  for (int j=averaging-c; j<averaging; j++){
    for (int i=0; i<16; i++){
      analog_data[i][j]=analogRead(A0+i);
    }
  }
  readAnalog();
}

void readAnalog(){
  for (int i=0; i<16; i++){
    int data=analogRead(A0+i);
    long S=data;
    for (int j=0; j<averaging-1; j++){
      analog_data[i][j]=analog_data[i][j+1];
      S+=analog_data[i][j];
    }
    analog_data[i][averaging-1]=data;
    analog_data[i][averaging]=S>>shift;
  }
}

void setLedst(int i_disp, int st){
  int id;
  id = st_leds[i_disp][st];
  lc.setLed(4, 4+id/8, id%8, true);
  id = st_leds[i_disp][1-st];
  lc.setLed(4, 4+id/8, id%8, false);
}

void setErr(int i_disp){
  int i_7;
  for (int i=0; i<3; i++){
    i_7=displays[i_disp][i];
    lc.setDigit(i_7/8,i_7%8,14,false);
  }
}

void setInt(int val, int i_disp) {
  int i_7, i_v;
  if (val>999){
    val=999;
  }
  for (int i=0; i<3; i++){
    i_7=displays[i_disp][i];
    i_v=val%10;
    val=int(val/10);
    lc.setDigit(int(i_7/8),i_7%8,i_v,false);
  }
}

void setFloat(int val, int i_disp) {
  int i_7, i_v;
  for (int i=0; i<3; i++){
    i_7=displays[i_disp][i];
    i_v=val%10;
    val=int(val/10);
    if (i==1){
      lc.setDigit(int(i_7/8),i_7%8,i_v,true);
    }
    else{
      lc.setDigit(int(i_7/8),i_7%8,i_v,false);  
    }
  }
}

void int_handler(){
  ++counter;
}

void getDallasTemps(){
    for (int i=0; i<oneWirePinsCount; i++) {
    sensor[i].requestTemperatures();
    temps[i]=sensor[i].getTempCByIndex(0);
    }
}

void configTSM_byD(int i_tsm){
  getDallasTemps();
  int u_p, u_tsm;
  float u_tsm0;
  int temp=temps[0];
  for (int i=0; i<tsmCount; i++){
    u_p=analog_data[TSMs[i][0]][averaging];
    u_tsm=analog_data[TSMs[i][2]][averaging]-analog_data[TSMs[i][1]][averaging];
    u_tsm0=u_tsm/(1+temps[0]*tks);
    TSM_ratios[i]=u_tsm0/u_p;
  }
}


int get_TSM_temps(){
    int u_p, u_tsm;
    float u_tsm0;
    for (int i=0; i<tsmCount; i++) {
    u_p=analog_data[TSMs[i][0]][averaging];
    u_tsm=analog_data[TSMs[i][2]][averaging]-analog_data[TSMs[i][1]][averaging];
    u_tsm0=u_p*TSM_ratios[i];
    temps[i]=(u_tsm-u_tsm0)/(tks*u_tsm0);
    }
}

boolean check_conf_timeout(int but, int timeout){
  for(int i=0; i<timeout; i++){
      if (digitalRead(but)){
        return false;
      }
      delay(100);
    }
    return true;
  }

void conf_var_res(){
  while(not(check_conf_timeout(conf_m_pin, 15))){
    readAnalog();
    for (int i=0; i<12; i++){
      //setInt(analogRead(A14),i);
      setInt(analog_data[i][averaging],i);
    }
  }
}

boolean displayData()    {
    boolean res=true;
    for (int i=0; i<5; i++){
      if (temps[i]>max_temps[i]){
        res=false;
        leds[temps_map[i]]=0;
      }
      if (temps[i]< 100){
        setFloat(int(temps[i]*10), temps_map[i]);
      }else{
        setInt(int(temps[i]), temps_map[i]);
      }

      if (pressures[i]>max_pressures[i]){
        res=false;
        leds[press_map[i]]=0;
      }
      if (pressures[i]< 100){
        setErr(press_map[i]);
        //setFloat(int(pressures[i]*10), press_map[i]);
      }else{
        setInt(int(pressures[i]), press_map[i]);
      }
    }
    setInt(counter%1000,0);
    setInt(counter/1000,3);
    return res;
  }

void setup() {

  attachInterrupt (0, int_handler, RISING);
  attachInterrupt (1, int_handler, RISING);

  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(8,is_ok);
  digitalWrite(10,is_ok);
  digitalWrite(12,is_ok);
  pinMode(conf_m_pin, INPUT_PULLUP);
  pinMode(conf_1_pin, INPUT_PULLUP);
  pinMode(conf_2_pin, INPUT_PULLUP);
  
  Serial.begin(9600);
  Serial.println("Smartex dev");

  analogReference(INTERNAL2V56);
  
 //Led displays initialization 
 for (int i=0; i<6; i++){
    lc.shutdown(i,false);
    /* Set the brightness to a medium values */
    lc.setIntensity(i,8);
    /* and clear the display */
    lc.clearDisplay(i);
  }

  // Start up the library on all defined bus-wires
  DeviceAddress deviceAddress;
  for (int i=0; i<oneWirePinsCount; i++) {;
    //ds18x20[i].setPin(oneWirePins[i]);
    sensor[i].setOneWire(&ds18x20[i]);
    sensor[i].begin();
    if (sensor[i].getAddress(deviceAddress, 0)) sensor[i].setResolution(deviceAddress, 10);
  }
  if (check_conf_timeout(conf_m_pin, 15)){

    conf_var_res();
    readAnalogCount(averaging);
    for (int i=0; i<tsmCount; i++){
      configTSM_byD(i);
    }
    for (int i=0; i<tsmCount; i++){
      EEPROM_float_write(i*4, TSM_ratios[i]);
    }
  }else{
    for (int i=0; i<tsmCount; i++){
      TSM_ratios[i]=EEPROM_float_read(i*4);
    }
  }
  readAnalogCount(averaging);
  is_ok = displayData();
}


void loop() {

  digitalWrite(8,is_ok);
  digitalWrite(10,is_ok);
  digitalWrite(12,is_ok);
  readAnalog(); 
  if (check_conf_timeout(conf_m_pin,15)){
     temp_source = not(temp_source);
  }
  if (temp_source){
    get_TSM_temps();
  }else{
    getDallasTemps();
  }
    for (int i=0; i<pressureCount; i++) {
      pressures[i]=analog_data[12+i][averaging];
    }
    ++measure_n;

    is_ok = displayData() && is_ok;
    for (int i=0; i<12; i++){
      setLedst(i,leds[i]);
    }
    //  delay(1000);
}

