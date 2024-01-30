#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h" 
MAX30105 particleSensor;
LiquidCrystal_I2C lcd(0x27,16,2);
hw_timer_t *My_timer = NULL;

int count = 0;
double aveRed = 0; //Thanh phan DC cua tin hieu red
double aveIr = 0;  //Thanh phan DC cua tin hieu IR
double sumIrRMS = 0; //Tong binh phuong IR
double sumRedRMS = 0; //Tong binh phuong tin hieu red
unsigned int i = 0; 
#define SUM_CYCLE 100 //tinh spo2 theo khoang thoi gian lay mau (o day ta khoi tao la 100)
int Num = SUM_CYCLE ;
double eSpO2 = 95.0; // gia trị spo2 thiet lap ban dau
double fSpO2 = 0.7; //he so loc
double fRate = 0.95; // bo loc thong thap cho gia tri IR/LED do de loai bo thanh phan AC
long irValue;
float ir_forGraph;
float red_forGraph;
double Ebpm;
#define TIMETOBOOT 3000
#define SCALE 88.0
#define SAMPLING 1
#define FINGER_ON 50000 //neu tin hieu hong ngoai thap hon 50000 co nghia rang tay khong cham vao cam bien
#define MINIMUM_SPO2 80.0
#define MAX_SPO2 100.0
#define MIN_SPO2 80.0
void sleepSensor() {
  byte ledBrightness = 0;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 200;
  int pulseWidth = 411;
  int adcRange = 16384; 
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void initSensor() {
  byte ledBrightness = 0x0A; //thiet lap bo phat tin hieu hong ngoai cua sensor (day la thu quyet dinh den viec do co chinh xac hay khong)
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 200; //thiet lap toc do lay mau la 200 mau/giay
  int pulseWidth = 411; //chieu rong xung: 411 microseconds
  int adcRange = 16384; //pham vi adc la 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void setup()
{
  My_timer = timerBegin(0, 80, true);//khoi tao timer voi prescaler la 80
  timerAttachInterrupt(My_timer, &onTimer, true); //gan ham ontimer lam ham thuc hien interrupt, khi nao
  timerAlarmWrite(My_timer, 1000000, true); //thiet lap thoi gian cho interrupt (o day ta thiet lap la 1000000 micro giay = 1s)
  timerAlarmEnable(My_timer); //ham kich hoat interrupt
  pinMode (4, OUTPUT);
  lcd.init();

  lcd.backlight();
  Serial.begin(115200);
  Serial.println("Initializing...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX3010X was not found.");

    Serial.println("Go to sleep. Bye");
    esp_deep_sleep_start();
  }

  initSensor();

}


#define FINGER_ON 50000 
#define LED_PERIOD 100 
#define MAX_BPS 180
#define MIN_BPS 45
double HRM_estimator( double fir , double aveIr) //tinh toan nhip tim va SpO2
{
    // he so loc cho BPM uoc luong
  static double fbpmrate = 0.95; 

  //bien thoi gian dung de xac dinh thoi gian cua cac diem chuyen muc
  static uint32_t crosstime = 0; 
  static uint32_t crosstime_prev = 0;

  //bien luu tru bpm hien tai va bpm da loc
  static double bpm = 60.0;
  static double ebpm = 60.0;

  //bien cho bo loc thong thap cua tin hieu IR
  static double eir = 0.0; 
  static double firrate = 0.85;
  static double eir_prev = 0.0;

  //ap dung bo loc thap cho tin hieu IR
  eir = eir * firrate + fir * (1.0 - firrate); 

  //kiem tra su chuyen doi qua muc trung binh
  if ( ((eir - aveIr) * (eir_prev - aveIr) < 0 ) && ((eir - aveIr) < 0.0)) { 
    crosstime = millis();

    //kiem tra giua 2 diem chuyen muc co phai la mot nhip tim hop le
    if ( ((crosstime - crosstime_prev ) > (60 * 1000 / MAX_BPS)) && ((crosstime - crosstime_prev ) < (60 * 1000 / MIN_BPS)) ) {
      //tinh toan BPM dua tren khoang thoi gian giua 2 diem chuyen muc
      bpm = 60.0 * 1000.0 / (double)(crosstime - crosstime_prev) ; 
      //ap dung bo loc thong thap de lam min gia tri BPM
      ebpm = ebpm * fbpmrate + (1.0 - fbpmrate) * bpm;

    } else {
    }
    crosstime_prev = crosstime;
  }
  eir_prev = eir;

  return (ebpm);
}

unsigned int loopCnt = 0;
void loop()
{
  count++;
  irValue = particleSensor.getIR();
  uint32_t ir, red ; // khai bao bien luu tru du lieu tho tu cam bien
  double fred, fir; // bien luu tru kieu so thuc cho gia tri RED va IR
  double SpO2 = 0; // bien luu gia tri spo2 truoc khi loc thong thap 


  particleSensor.check();

  while (particleSensor.available()) {
    red = particleSensor.getFIFOIR(); //doc gia tri RED
    ir = particleSensor.getFIFORed(); //doc gia tri IR
    i++; loopCnt++;
    fred = (double)red;
    fir = (double)ir;
    //tinh gia tri trung binh co loc qua bo loc low_pass
    aveRed = aveRed * fRate + (double)red * (1.0 - fRate);
    aveIr = aveIr * fRate + (double)ir * (1.0 - fRate); 
    //tinh tong binh phuong sai so cho RED va IR
    sumRedRMS += (fred - aveRed) * (fred - aveRed); 
    sumIrRMS += (fir - aveIr) * (fir - aveIr);
    //uoc luong gia tri nhip tim
    Ebpm = HRM_estimator(fir, aveIr);

    //dieu chinh toc do va ve do thi
    if ((i % SAMPLING) == 0) {
      if ( millis() > TIMETOBOOT) {
        //tinh toan gia tri cho do thi 
        ir_forGraph = 2.0 * (fir - aveIr) / aveIr * SCALE + (MIN_SPO2 + MAX_SPO2) / 2.0;
        red_forGraph = 2.0 * (fred - aveRed) / aveRed * SCALE + (MIN_SPO2 + MAX_SPO2) / 2.0;
        if ( ir_forGraph > MAX_SPO2) ir_forGraph = MAX_SPO2;
        if ( ir_forGraph < MIN_SPO2) ir_forGraph = MIN_SPO2;
        if ( red_forGraph > MAX_SPO2 ) red_forGraph = MAX_SPO2;
        if ( red_forGraph < MIN_SPO2 ) red_forGraph = MIN_SPO2;
        //kiem tra xem ngon tay co cham vao cam bien khong
        if ( ir < FINGER_ON) eSpO2 = MINIMUM_SPO2;
      }
    }

      print_Serial();

      // if (count >50){
      //   LCD_print();
      //   count =0;
      // }


    if ((i % Num) == 0) {
      double R = (sqrt(sumRedRMS) / aveRed) / (sqrt(sumIrRMS) / aveIr);

    //cong thuc tinh gia tri SPO2
#ifdef MAXIMREFDESIGN

      SpO2 = -45.060 * R * R + 30.354 * R + 94.845 ;

#else
#define OFFSET 0.0
      SpO2 = -23.3 * (R - 0.4) + 100 - OFFSET ;
      if (SpO2 > 100.0 ) SpO2 = 100.0;
#endif
//ap dung bo loc thong thap de lam mịn gia tri cho du lieu SPO2
      eSpO2 = fSpO2 * eSpO2 + (1.0 - fSpO2) * SpO2; //bo loc thong thap


      sumRedRMS = 0.0; sumIrRMS = 0.0; i = 0;
      break;
    }
    particleSensor.nextSample(); //chuyen sang mau tiep theo
  }
}
void print_Serial(){
  if (irValue < 5000) {
    Serial.println("No Finger");
    digitalWrite(4, LOW);
  }
  else {
        Serial.print(Ebpm);
        Serial.print(",");
        Serial.print(ir_forGraph); 
        Serial.print(","); 
        Serial.print(red_forGraph);
        Serial.print(","); 
        Serial.println(irValue);
        if (Ebpm >130 || ir_forGraph < 90 || Ebpm < 80)    digitalWrite(4, HIGH); //khi gia tri nhip tim nam ngoai muc thi se bat den led, thiet lap o chan GPIO 4
        else digitalWrite(4, LOW);
  }
}
void IRAM_ATTR onTimer(){  //dinh nghia chuc nang cua interrupt
  if (irValue < 5000){
    lcd.clear();
    lcd.setCursor(4,0);
    lcd.print("MAX30102");
    lcd.setCursor(4,1);
    lcd.print("NoFinger");
  }
  else{
    lcd.clear();
    lcd.setCursor(4,0);
    lcd.print("MAX30102");
    lcd.setCursor(0,1);
    lcd.print("BPM=");
    lcd.setCursor(4,1);
    lcd.print(Ebpm);
    lcd.setCursor(8,1);
    lcd.print("SpO2=");
    lcd.setCursor(13,1);
    lcd.print(ir_forGraph);
  }
}