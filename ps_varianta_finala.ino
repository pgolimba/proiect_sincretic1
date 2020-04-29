#define TRIGGER_VALUE (0x10)
#define ECHO_VALUE (0x20)
#define DIST_INF (9999)

#define MOTOR_IN1 (0x10)
#define MOTOR_IN2 (0x20)
#define MOTOR_IN3 (0x40)
#define MOTOR_IN4 (0x80)
#define MOTOR_SPEED (95) // Intre [0 - 255]

#define IR_PIN (0x04)
#define IR_THR (512)

#define BAUD (9600L) // Merge pana la 28800 cu implementarea actuala
#define MYUBRR (((F_CPU / 16) / BAUD) - 1) // Valoare baud rate de scris in registru
#define UART_DELAY_US (((1000000L / BAUD) * 10))

long timer1_ovf = 0;
int state = 0;

ISR(TIMER1_OVF_vect)
{ 
  timer1_ovf++;
}

void waitUS(unsigned long microsec)
{
  unsigned long time = micros();
  while(micros() - time < microsec)
  {
    // Asteapta...
  }
}

 
void waitMS(unsigned long milisec)
{
  unsigned long time = millis();
  while(millis() - time < milisec)
  {
    // Asteapta...
  }
}





void clearRegisters() {
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR2A = 0;
  TCCR2B = 0;

 // ADMUX = 0;
}

/*----------CONFIGURARE MOTOR---------*/
void motorInit() {
  noInterrupts(); // Dezactiveaza toate intreruperile

  DDRD |= 0xF0;  // Pin 4,5,6,7 iesire pentru in1, in2, in3, in4 directie motor
  DDRD |= 0x08;  // Pin  3 - PWM output A 
  DDRB |= 0x08;  // Pin 11 - PWM output B
  
  TCCR2A |= (1 << WGM21) | (1 << WGM20);   // Setez fast pwm mode
  TCCR2A |= (1 << COM2A1) | (1 << COM2B1); // Activez ambele canale A=pin 3 si B=pin 11
  TCCR2B |= (1 << CS22) | (1 << CS20);     // Setez prescaler (128)
  OCR2A = MOTOR_SPEED; // Viteza motorului drept
  OCR2B = MOTOR_SPEED; // Viteza motorului stang

  interrupts();
}

/*---------END CONFIGURARE MOTOR---------*/
 
//void irInit() {
//  noInterrupts();
//  ADCSRA |= (1 << ADEN);
//  interrupts();
//}
/*-----------CONFIGURARE TIMER-----------*/
void timer1(){
  noInterrupts();
  DDRB |= TRIGGER_VALUE; // Pin 12 - output Trig
  DDRB &= ~ECHO_VALUE;   // Pin 13 - input Echo               
  TCCR1B |= (1 << CS10);  // Fara prescaler (1)
  TIMSK1 |= (1 << TOIE1); //Seteaza intreruperea 
  interrupts();
}
/*-----------END CONFIGURARE TIMER-----------*/

/*-------------SENZOR DISTANTA------------*/


float sensor_HCSR04(){
  float timp, distanta;

  TCNT1 = 0;
  PORTB &= ~TRIGGER_VALUE;   //sterge trigger
  waitUS(2);
  PORTB |= TRIGGER_VALUE;   // activare semnal trigger 1 logic
  waitUS(10);
  PORTB &= ~TRIGGER_VALUE; // sterge trigger

  TCNT1 = 0;
  timer1_ovf = 0;

  while(!(PINB & ECHO_VALUE)); // Asteapt dupa ECHO pe 1
  while(PINB & ECHO_VALUE); // Cat timp ECHO e pe 1, timer1 va numara

  timp = (((timer1_ovf * 65535) + TCNT1) * 0.0625f); // Timpul  in secunde 
  distanta = ((timp * 0.0343f) / 2) - 8;

  if(distanta > 1 && distanta < 15) {
    return distanta;
  } else {
    return DIST_INF;
  }
}
/*---------FINAL SENZOR DISTANTA----------*/


/*---------MOTOR-------*/
void motorStop() {
  PORTD &= ~0xF0;
}

void motorFata(){
  motorStop();
  PORTD |= MOTOR_IN1;
  PORTD |= MOTOR_IN3;
}

void motorSpate(){
  motorStop();
  PORTD |= MOTOR_IN2;
  PORTD |= MOTOR_IN4;
}

void motorStanga(){

  motorStop();
  PORTD |= MOTOR_IN1;
  PORTD |= MOTOR_IN4;
}

void motorDreapta(){
  motorStop();
  PORTD |= MOTOR_IN2;
  PORTD |= MOTOR_IN3;
}
/*--------FINAL MOTOR---------*/

/*--------SENZOR DE LINIE------*/
int trackingSensor() {
  int irVal = (PIND & IR_PIN);
  if(irVal == IR_PIN) {
    PORTB &= ~0x04;
    return 0; // negru
  } else {
    PORTB|=0x04;
    return 1; // alb
    
  }
}
/*--------FINAL SENZOR LINIE-------*/



void setup() {
  clearRegisters();
 // UART_Init(MYUBRR);
  motorInit();
  timer1();
  waitMS(3000);
DDRB|=0x04;
}

void loop(){
  //PORTB|=0x04;
  int irVal = trackingSensor();
  float distanta = sensor_HCSR04();



  if(state == 0) {
    if(irVal == 0) {
      state = 1;
    } else {
      state = 10;
    }
  } else if (state == 1) {
    motorStanga();
    if(distanta != DIST_INF) {
      motorStop();
      state = 2;
    } else {
      state = 1;
    }
  } else if (state == 2) {
    if(distanta != DIST_INF) {
      motorFata();
      state = 3;
    } else {
      state = 1;
    }
  } else if (state == 3) {
    if(irVal == 1) {
      motorStop();
      state = 4;
    } else {
      state = 3;
    }
  } else if (state == 4) {
    if(irVal == 1) {
     // motorStop();
     motorSpate();
      //motorStop();
      state = 4;
    } else {
      motorStop();
      state = 0;
    }
  } else if(state == 10) {
motorStop();  }

  waitMS(100);
}
