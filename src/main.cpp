#include <Arduino.h>

// void setup() {
//   // put your setup code here, to run once:
//   pinMode(0, OUTPUT);
//   pinMode(1, OUTPUT);
//   analogWrite(0, 128);
//   analogWrite(1, 128);
// }
#define PWM_MIN 0
#define PWM_MAX 511

void setup()
{

  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);

  Serial.begin(19200);

  // TCCR1D=0b00010001;
  // pinMode(10,OUTPUT);
  TCCR1A = 0; //reset the register
  TCCR1B = 0; //reset the register
  TCNT1 = 0;
  TCCR1A = 0b10100010; //COM1A0,COM1B0 are 0, COM1A1, COM1B1 are 1
  //also WGM11=1, WGM10=0 (8bit PWM/ TOP=0xFF)
  //TCCR1B = 0b00001001; //WGM13 0 and WGM12 is 1, no prescaler CS10 is 1
  // TCCR1B=0b00000011;//WGM13 and WGM12 are 0 with 1024 prescaler CS10 and CS12 is 1
  // OCR1A=0;// duty cycle value
  // OCR1B=500;// duty cycle value
  // for (int i = 10; i < PWM_MAX; i++) {
  //   OCR1A = i;
  //   OCR1B = i-5;
  //   delay(100);
  // }
}

#define SAMPLES 5
#define CURRENT_MAX 12
#define CURRENT_FAULT 20 / CURRENT_ADC_SCALE
#define CURRENT_CAL -0.28
#define CURRENT_MIN 0.5
// #define VOUT_MAX 14.5

#define CURRENT_TOL 0.1
#define VOLTAGE_TOL 0.01

#define VOLTAGE_MIN 10

// V Ref = 5V
#define CURRENT_ADC_SCALE 0.176587302

// V Ref = 5V, 47:6.8 (6.9) divider
// #define VOLTAGE_IN_ADC_SCALE 0.03333
// #define VOLTAGE_OUT_ADC_SCALE 0.03333
#define VOLTAGE_IN_ADC_SCALE 0.03885
#define VOLTAGE_OUT_ADC_SCALE 0.03752
// 0.013901654

#define VOLTAGE_LOW 13.8
#define VOLTAGE_HIGH 14.7

int pwm = 0;

#define setPwm(v) \
  pwm = (v);      \
  OCR1A = pwm;

void stopPwm()
{
  setPwm(0);
  // TODO make sure that line is low
  //TCCR1B = 0; //disable pwm;
  // digitalWrite(9, LOW);
}

void startPwm()
{
  setPwm(0);
  TCCR1B = 0b00001001;
}

void incPwm()
{
  if (pwm < PWM_MAX)
  {
    // TCCR1B = 0b00001001;
    setPwm(pwm + 1);
  }
}

void decPwm()
{
  if (pwm > 0)
  {
    setPwm(pwm - 1);
  }
  else
  {
    // stopPwm();
    setPwm(0);
  }
}

long cnt = 0;
int ptr = 0;
boolean on = true;
boolean oc = false;
boolean idle = true;

int iout_samples[SAMPLES];
int vout_samples[SAMPLES];
int vin_samples[SAMPLES];

float max_pwr = 0;
int max_pwr_pwm = 0;

float vout_max = VOLTAGE_LOW;
float iout_max = 7.5;
bool high_out = 0;

long wait = 1000;
long trickle = 0;

void loop()
{
  int iout_adc = analogRead(A1) - analogRead(A0);

  if (iout_adc >= (int)(CURRENT_FAULT))
  {
    oc = true;
    wait = 2000;
    stopPwm();
  }

  iout_samples[ptr] = iout_adc;
  vin_samples[ptr] = analogRead(A3);
  vout_samples[ptr] = analogRead(A2);
  ptr++;
  if (ptr >= SAMPLES)
  {
    ptr = 0;
  }

  float iout_avg = 0;
  float vout_avg = 0;
  float vin_avg = 0;

  for (int i = 0; i < SAMPLES; i++)
  {
    iout_avg += iout_samples[i];
    vout_avg += vout_samples[i];
    vin_avg += vin_samples[i];
  }

  float iout = (iout_avg / SAMPLES) * CURRENT_ADC_SCALE + CURRENT_CAL;
  float vout = (vout_avg / SAMPLES) * VOLTAGE_OUT_ADC_SCALE;
  float vin = (vin_avg / SAMPLES) * VOLTAGE_IN_ADC_SCALE;

  float pwr = vout * iout;

  if (vout_max == VOLTAGE_LOW && iout > CURRENT_MIN)
  {
    vout_max = VOLTAGE_HIGH;
  }
  else if (vout_max == VOLTAGE_HIGH && iout < CURRENT_MIN)
  {
    if (trickle==0) {
      trickle = 1000000;
    }
    else if (trickle > 0)
    {
      trickle--;
    }
    else
    {
      vout_max = VOLTAGE_LOW;
    }
  }

  if (wait > 0)
  {
    wait--;
  }
  else if (wait == 0)
  {
    oc = 0;
    wait = -1;
    startPwm();
  }
  else
  {
    if ((vin<VOLTAGE_LOW +VOLTAGE_TOL || vout + VOLTAGE_TOL > vout_max) && iout < 0.2)
    {
        stopPwm();
        vout_max = VOLTAGE_LOW;
        wait = 1000;
    }
    else
    {
      if (vout > vout_max + 0.05 || iout > iout_max + 0.05 || vout > vin)
      {
        decPwm();
      }
      else if (vout < vout_max && iout < iout_max && vin > vout) //vin > vout_max &&
      {
        incPwm();
      }
    }
  }

  // if (pwr > max_pwr)
  // {
  //   max_pwr = pwr;
  //   max_pwr_pwm = pwm;
  // }
  // else if (cnt % 1000 == 0)
  // {
  //   max_pwr = 0;
  //   max_pwr_pwm = 0;
  // }

  boolean print_request = false;

  if (Serial.available() > 0)
  {
    char c = Serial.read();
    switch (c)
    {
    case 10:
      print_request = true;
      break;
    case 'u':
      Serial.println("# Current limit inc");
      if (iout_max < CURRENT_MAX)
        iout_max += 0.1;
      break;
    case 'd':
      Serial.println("# Current limit dec");
      if (iout_max > 0)
      {
        iout_max -= 0.1;
        if (iout_max < 0)
          iout_max = 0;
      }
      break;
    case '0':
      Serial.println("# Charger off");
      wait = -1;
      stopPwm();
      break;
    case '1':
      Serial.println("# Charger on");
      startPwm();
      break;
    }
  }

  if (print_request || cnt % 250 == 0)
  {
    print_request = false;
    Serial.print("v_in:");
    Serial.print(vin);
    Serial.print(" ");
    Serial.print("v_out:");
    Serial.print(vout);
    Serial.print(" ");
    Serial.print("i_out:");
    Serial.print(iout);
    Serial.print(" ");
    Serial.print("i_max:");
    Serial.print(iout_max);
    // Serial.print(" ");
    // Serial.print("i_max_t:");
    // Serial.print(iout_max+CURRENT_TOL);
    Serial.print(" ");
    Serial.print("duty:");
    Serial.print((float)pwm * 10 / PWM_MAX);
    Serial.print(" ");
    Serial.print("pwm_on:");
    Serial.print(TCCR1B & 1);
    Serial.print(" ");
    Serial.print("pwr:");
    Serial.print(pwr);
    // Serial.print(" ");
    // Serial.print("p_max:");
    // Serial.print(max_pwr);
    Serial.print(" ");
    Serial.print("oc:");
    Serial.print(oc ? "1" : "0");

    Serial.println();
    delay(2);
  }

  cnt++;
}
