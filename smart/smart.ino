#define averaging 128
#define  shift  7
#define conf_m_pin 6


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

int b_leds[]={1,2,3,4,5,6,7,8,11,12,13,14,15,16,17,18,19,20,21,22,23};
int b_leds_v[]={0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1};

int button_in[]={36,37,42,43};
int button_out[]={15,16,17,18};

int butt_leds_map[]={19,7,0,17,20,13,0,16,10,12,0,15,6,18,0,14};

int buttons_val;

const int oneWirePinsCount=3;
const int tsmCount=4;
const int pressureCount=4;
const float tks=0.00428;
float TSM_ratios[4]={1, 1, 1, 1};
float temps[]={0,0,0,0,0};
float err_temps[]={0,0,0,0,0};
int max_temps[]={50,50,50,50,50};
int min_start_temps[]={25,0,0,0,0};
int skipped_temps[]={0,0,0,0,1};
float pressures[]={0,0,0,0,0};
float err_pressures[]={0,0,0,0,0};
int max_pressures[]={512,512,512,512,512};
int skipped_press[]={0,0,0,0,1};
int leds[]={1,1,1,1,1,1,1,1,1,1,1,1};
int temps_map[]={11, 8, 10, 4, 9};
int press_map[]={5, 7, 1, 6, 2};
//const int reset_pin=16;
//const int check_pin=18;
boolean temp_source=true;
boolean is_ok=false;
boolean heating = false;
boolean compressor = false;
boolean separator = false;
boolean start_approved = false;

int analog_data[16][averaging+1];

int TSMs[4][3]={{10, 9, 8},{4, 7, 6},{11,2,3},{5,1,0}};
OneWire ds18x20[]={OneWire(21),OneWire(20), OneWire(19)};
DallasTemperature sensor[oneWirePinsCount];

unsigned long counter=0;
unsigned long err_counter=0;
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

void set_is_ok(boolean status){
  if (is_ok !=status){
    if (status){
      switch_b_status(2, 3);
    }else
    {
      switch_b_status(3, 2);
    }
    is_ok = status;
  }
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

void setLedsb(int i_led, int st){
  lc.setLed(5,i_led/8,i_led%8, st);
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
  int i_in=but/4;
  int i_out=but%4;
  digitalWrite(button_in[i_in],0);
  for(int i=0; i<timeout; i++){
      if (digitalRead(button_out[i_out])){
        digitalWrite(button_in[i_in],1);
        return false;
      }
      delay(100);
    }
    digitalWrite(button_in[i_in],1);
    return true;
  }

void conf_var_res(){
  while(not(check_conf_timeout(conf_m_pin, 15))){
    readAnalog();
    for (int i=0; i<12; i++){
      setInt(analog_data[i][averaging],i);
    }
  }
}

boolean displayData(boolean actual)    {
    boolean res=true;
    float temp, pressure;
    for (int i=0; i<5; i++){
      if (actual){
        temp=temps[i];
        pressure=pressures[i];
      }else{
        temp=err_temps[i];
        pressure=err_pressures[i];
      }
      if (not(skipped_temps[i])){
      if (actual&&temp>max_temps[i]){
        res=false;
        leds[temps_map[i]]=0;
      }
      if (temp<-50){
        setErr(temps_map[i]);
        res=false;
        skipped_temps[i]=1;
        leds[temps_map[i]]=0;
      }
      else if (temp< 100){
        setFloat(int(temp*10), temps_map[i]);
      }else{
        setInt(int(temps), temps_map[i]);
      }
      }

      if (not(skipped_press[i])){
      if (actual&&pressure>max_pressures[i]){
        res=false;
        leds[press_map[i]]=0;
      }
      if (pressure< 100){
        setErr(press_map[i]);
        res=false;
        skipped_press[i]=1;
        leds[press_map[i]]=0;
        //setFloat(int(pressures[i]*10), press_map[i]);
      }else{
        setInt(int(pressure), press_map[i]);
      }
      }
    }
    if (actual){
      setInt(counter%1000,0);
      setInt(counter/1000,3);
    }else{
      setInt(err_counter%1000,0);
      setInt(err_counter/1000,3);
    }
    return res;
  }

void save_err(){
  for (int i=0; i<5; i++){
    err_temps[i]=temps[i];
    err_pressures[i]=pressures[i];
    err_counter=counter;
  }
}

void reset_error(){
  for (int i=0; i<5; i++){
    if (not(skipped_temps[i])){
      leds[temps_map[i]]=1;
    }
    if (not(skipped_press[i])){
      leds[press_map[i]]=1;
    }
  }
  set_is_ok(true);
}

int read_buttons(){
  int res=0;
  int i_butt=0;
  for (int i=0; i<4;i++){
    digitalWrite(button_in[i], 0);
    for (int j=0; j<4; j++){
      i_butt=i*4+j;
      res+= (1-digitalRead(button_out[j]))<<i_butt;
    }
    digitalWrite(button_in[i], 1);
  }
  return res;
}

void indicate_buttons(int val){
  for (int i=0; i<16; i++){
    if (val & (1<<i)){
      setLedsb(b_leds[butt_leds_map[i]],true);
    }
  }
}

void reset_indication(){
  for (int i=0; i<21; i++){
    setLedsb(b_leds[i],b_leds_v[i]);
  }
}

void switch_b_status(int b1, int b2){
  b_leds_v[b1]=1;
  b_leds_v[b2]=0;
  
}

void handle_buttons(int val){
  boolean pr_butt = false;
  if (val& (1<<4)){
    heating=false;
    switch_b_status(20,10);
  }
  if (val& (1<<13)){
    compressor=false;
    switch_b_status(18,19);
  }
  if (val& (1<<7)){
    separator=false;
    switch_b_status(16,17);
  }
  
  if (val& (1<<8)){
    if (is_ok){
      heating=true;
      switch_b_status(10,20);
    }
  }
  if (val & (1<<0)){
    if (is_ok && start_approved){
      compressor=true;
      switch_b_status(19,18);
    }
  }
  if (val & (1<<3)){
    if (is_ok && start_approved){
      separator=true;
      switch_b_status(17,16);
    }
  }
  if (val & (1<<15)){
    if (is_ok){
      start_approved=true;
      switch_b_status(14,15);
    }
  }
  if (val & (1<<11)){
    if (is_ok){
      start_approved=false;
      switch_b_status(15,14);
    }
  }
  if (val & (1<<9)){
    for (int i=0; i<6; i++){
      for (int j=0; j<8; j++){
        lc.setDigit(i,j,8,true);
      }
    }
    while(check_conf_timeout(9, 10)){
      
    }
  }
}

boolean check_start_cond(){
  for (int i=0; i<5; i++){
    if (not(skipped_temps[i]) && min_start_temps[i]>temps[i]){
      return false;
    }
  }
  switch_b_status(4, 5);
  return true;
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
  for (int i=0; i<4; i++){
    pinMode(button_out[i], INPUT_PULLUP);
    pinMode(button_in[i], OUTPUT);
  }
  //pinMode(check_pin, INPUT_PULLUP);
  //pinMode(reset_pin, INPUT_PULLUP);
  
  Serial.begin(9600);
  Serial.println("Smartex dev");

  analogReference(INTERNAL2V56);
  
 //Led displays initialization 
 for (int i=0; i<6; i++){
    lc.shutdown(i,false);
    /* Set the brightness to a max values */
    lc.setIntensity(i,15);
    /* and clear the display */
    lc.clearDisplay(i);
  }

  // Start up the library on all defined bus-wires
  DeviceAddress deviceAddress;
  for (int i=0; i<oneWirePinsCount; i++) {;
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
  set_is_ok( displayData(true));
}

void loop() {

  if (not(start_approved)&&is_ok){
    start_approved = check_start_cond();
  }
  buttons_val=read_buttons();
  Serial.println(buttons_val); //should be deleted in prod
  if (buttons_val){
    indicate_buttons(buttons_val);
  }else{
    reset_indication();
  }
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
  
  handle_buttons(buttons_val);
  
  boolean disp_res;
  
  // +++++++++Check error values ++++++++++
  if (not(buttons_val & (1<<1))){
    disp_res=displayData(true);
  }else
  {
    disp_res=displayData(false);
  }
  if (is_ok){
    if (!disp_res){
      set_is_ok(false);
      save_err();
    }
  }else{
    
    // ++++++++ Reset Button pressed ++++++++++
    if (buttons_val & (1<<5)){
      reset_error();
    }
  }
    for (int i=0; i<12; i++){
      setLedst(i,leds[i]);
    }
  digitalWrite(8,is_ok && heating);
  digitalWrite(10,is_ok && compressor && start_approved);
  digitalWrite(12,is_ok && separator && start_approved);

  if (Serial.available()>0){

  char test= Serial.read();
  if (test=='d'){
  for (int i=0; i<4; i++) {
    Serial.print(temps[i]);
    Serial.print(" ");
    Serial.print(pressures[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
  }
  }
}

