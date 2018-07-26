const byte frameStartByte = 0x7E;
const byte frameTypeTXrequest = 0x10;
const byte frameTypeRXpacket = 0x90;
const byte frameTypeATcommand = 0x08;
const byte frameTypeATresponse = 0x88;
const long destAddressHigh = 0x13A200;
const long destAddressLow = 0x40A4D723;
char DBcommand[ ] = "DB";
bool automatic=false;
byte ATcounter=0; // for identifying current AT command frame
byte rssi=0; // RSSI value of last received packet

bool RXpacket=false;
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define BAUD_RATE 9600
#define BAUD_RATE_DIVISOR (F_CPU / 16 / BAUD_RATE - 1)

volatile int contador=0;
int sensorPin = 0; //the analog pin the TMP36's Vout (sense) pin is connected to
float distanceLeft=100.0;
float distanceFront=100.0;
/*
* Brushed_H_Bridge sketch
* commands from serial port control motor speed and direction
* digits '0' through '9' are valid where '0' is off, '9' is max speed
* + or - set the direction
*/

const unsigned int precision=(1000000/(F_CPU/1000))*8;  // constant for conversion of counted delay to distance in cm
const unsigned int ns2cm=58000;
volatile unsigned int start=0;
volatile unsigned long atraso=0;
volatile byte direccao=1; // guarda a direcção da contagem (1 é para cima)

volatile bool frente = true;
volatile unsigned long start2=0;
volatile unsigned long final2=0;
volatile unsigned int periodo2 = 0;
float durationFront = 0;
volatile int echoDone = 0;
bool trigger = false;

volatile unsigned int rate=0;

void USART_init(void) {
  UCSR0A = 0 << TXC0 | 0 << U2X0 | 0 << MPCM0; // Configure USART for: no Double Speed, Multi-processor Communication Mode disabled
  UCSR0B = 1 << RXCIE0 | 0 << TXCIE0 | 0 << UDRIE0 | 1 << RXEN0 | 1 << TXEN0 | 0 << UCSZ02 | 0 << TXB80; // Configure USART for: interrupts and receiver disabled, transmitter enabled, 8 bit character size
  UCSR0C = 0 << UMSEL01 | 0 << UMSEL00 | 0 << UPM01 | 0 << UPM00 | 0 << USBS0 | 1 << UCSZ01 | 1 << UCSZ00 | 1 << UCPOL0; // Configure USART for: Asynchronous operation, no parity, 1 stop bit, 8 bit character size

  //Baud Rate initialization
  UBRR0 = BAUD_RATE_DIVISOR;
}
void USART_transmit(char data)
{
  while((UCSR0A & (1<<UDRE0))==0x00); 
  UDR0 = data; //load data to UDR for transmission
}

ISR(USART_RX_vect) {
  char ch = UDR0; // read incoming byte to clear interrupt flag (ATmega328P datasheet, section 20.11)
  byte frametype = UDR0;

  if (ch == 0x7E)
  {
    contador = 0;
    contador++;
  }
  if (contador != 3)
  {
    contador++;
  }
  if (contador == 3 && frametype == frameTypeRXpacket)
  {
    RXpacket=true;
    contador++;
  }
   if (contador == 3 && frametype == frameTypeATresponse)
  {
    RXpacket=false;
    contador++;
  }
   if (contador == 8 && RXpacket==false)
  {
    rssi=ch;
  }
  if (contador == 15 && RXpacket==true)
  { 
    if(!automatic)  
    {
      if ( ch == 'W')
      { 
        if (!frente){
          PORTD ^= ( (1<<PORTD7) | (1<<PORTD6) );
          PORTB ^= ( (1<<PORTB5) | (1<<PORTB4) );
          frente = true;
        }     
        setSpeeds(255,255);
             
      }   
      else if  ( ch == 'S')
      {        
        
        if(frente){
          PORTD ^= ( (1<<PORTD7) | (1<<PORTD6) );
          PORTB ^= ( (1<<PORTB5) | (1<<PORTB4) );
          frente = false;
        }
        setSpeeds(255,255);
      }
      else if  ( ch == 'A')
      {     
        setSpeeds(130,255);     
      }
      else if  ( ch == 'D')
      {     

        setSpeeds(255,130);     
      }
      else if  ( ch == 'P')
      { 
        if (!frente){
          PORTD ^= ( (1<<PORTD7) | (1<<PORTD6) );
          PORTB ^= ( (1<<PORTB5) | (1<<PORTB4) );
          frente = true;
        }  
        automatic=true;   
            
      }else if  ( ch == 'O')
      { 
      automatic=false;   
      setSpeeds(0,0);   
      }
     }
    else if  ( ch == 'O')
    { 
      automatic=false;   
      setSpeeds(0,0);   
    }
 }
}

// ISR interrupt vector 
ISR(TIMER1_CAPT_vect)
{
  if(TCCR1B & (1<<ICES1)) // rising edge no Input Capture
  { 
    start = ICR1; // save the input capture value
    direccao=1; // o início do Echo pulse é sempre na contagem ascendente 
  }
  else{ // Falling edge    
    if (direccao==1){
      // ainda estava na fase ascendente da contagem
      atraso = ICR1 - start;
    }
    else{
      // já estava na fase descendente da contagem -> contabilizar a fase da subida
      atraso = 2*OCR1A - ICR1 - start;
    }
  }
  TCCR1B ^= 1<<ICES1; //Fazer o toggle do bit (0->1 ou 1->0) que desencadeia o Input Capture, ou seja o bit ICES1 (e apenas este bit!)
}

// ISR interrupt vector 
ISR(TIMER1_COMPA_vect)
{
  // Atingiu o topo da contagem. Vai iniciar a contagem decrescente
  direccao=0;
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER0_COMPA_vect)
{

    // Atingiu o topo da contagem. Vai iniciar a contagem decrescente
    periodo2++;
}

ISR(INT0_vect)
{
  
  if(trigger){

    trigger = false;

    echoDone = 1;
  }

  else {
      durationFront = OCR0A + periodo2*128;
      distanceFront= (durationFront*0.034/2) - 10.2;
      echoDone = 1;
      trigger = true;
      }
}



int leftSensor = 100;
int frontSensor = 100;

int frontLimit = 50;
int leftLimit = 50;

int LEFTMOTOR = 11;
int RIGHTMOTOR = 5; 


void setSpeeds(int left, int right){
  //analogWriting_left(LEFTMOTOR, left);
  analogWriting_left(LEFTMOTOR, left);
  
  //analogWriting_right(RIGHTMOTOR, right);
  analogWriting_right(RIGHTMOTOR, right);
}



void setup()
{
    our_delay(7000000);
    USART_init(); 
       
    // Inicializar aqui os pinos 4, 5 e 7 (PD4, PD5 e PD7) como OUTPUT   [LEFT MOTOR]    5 ENEABLE~
    DDRD= 0<<DDD2 | 1<<DDD5 | 1<<DDD7 | 1<<DDD6 | 1<<DDD3;
    DDRB= 1<<DDB2 | 0<< DDB0 | 1<<DDB3 | 1<<DDB4 | 1<<DDB5 | 1<<DDB1;  //PIN 11, 12, 13
    // Inicializar aqui os pinos 1, 2 e 2 (PD4, PD5 e PD7) como OUTPUT   [RIGHT MOTOR]    3 ENABLE~
    

    PORTD = 0<<PORTD6 | 1<<PORTD7 | 1<<PORTD2;
    
    PINB = 1<<PORTB4 | 0<<PORTB5;
  
    // initialize o modo 9 - PWM, Phase and Frequency Correct configurando WGM10, WGM11, WGM12 e WGM13 nos dois registos que se seguem (TCCR1A e TCCR1B)
    TCCR1A = 0<<COM1A1 | 0<<COM1A0 | 1<<COM1B1 | 0<<COM1B0 | 0<<WGM11 | 1<<WGM10; // OC1A OFF, OC1B ON. Configurar WGM11, WGM10
    TCCR1B = 1<<WGM13 | 0<<WGM12 | 0<<CS12 | 1<<CS11 | 0<<CS10 | 1<<ICES1; // configurar CS10, CS11, CS12 de forma a ter timer clock = F_CPU/8, configurar WGM12, WGM13
    TIMSK1= 1<<ICIE1 | 1<<OCIE1A; // activar as interrupções do input capture interrupt (ICIE1) e do Output Compare A Match  (OCIE1A)
    OCR1A=60000; // periodo = 60000us
    OCR1B=10; // 10us de pulso de trigger
  

    TCCR0A = 0<<COM0A1|0<<COM0A0| 0<<COM0B0 | 1<<WGM01 | 0<<WGM00; // OC1A OFF, OC1B ON. Configurar WGM11, WGM10
    TCCR0B = 0<<WGM02 | 0<<CS02 | 1<<CS01 | 0<<CS00; // configurar CS10, CS11, CS12 de forma a ter timer clock = F_CPU/8, configurar WGM12, WGM13
    TIMSK0 |= 1<<OCIE0A;
    OCR0A = 255;
    OCR0B = 10;
    
    EICRA |= 1<<ISC01 | 0<<ISC00;
    EIMSK |= 1<<INT0;

  
    ADMUX = 0<<REFS1 | 1<<REFS0 | 0<<ADLAR | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0;; // Voltage reference for ADC=AVCC (5.0V); configurar ADLAR para right adjust for 10 bit resolution, select ADC0 (pino A0),
    ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADATE | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0;  // Enable ADC (ADEN), Start ADC conversion (ADSC) , Enable Auto Trigger, enable ADC Interrupts,  configure ADPS2-0 for 128 prescale ( resulta numa frequência 16MHz/128=125kHz)
    ADCSRB = 0<<ACME; // free running mode, (leave ACME as 0).
    DIDR0=0x3F; // desliga digital inputs nos pinos ADC0-ADC5 para reduzir consumo
    
    asm("SEI"); 
}



ISR(ADC_vect)  // include the name of the interrupt vector
{
    rate = ADC;          // in this case 10 bits are being used but only needs to read the high value for 8 bit precision
    // REMEMBER: once ADCH is read the ADC will update
}

void loop()         
{
  formatTXAPIpacket(rate);
  formatATcommandAPI("DB"); 

  echoDone = 0;
  periodo2 = 0;
  
  //CAR
  PORTB &= ~(1<<PORTB1);
  our_delay(3);
  // Sets the trigPin on HIGH state for 10 micro seconds
  PORTB |= (1<<PORTB1);
  our_delay(7);
  PORTB &= ~(1<<PORTB1);
    
  while(echoDone==0);

  frontSensor = 200;
 
  our_delay(35000);
  distanceLeft = (atraso*precision)/ns2cm;   
  
  char key='nul';
  int frontLimit=35;

    //delay(1000);
    if(automatic)
    {   
        if(distanceFront<frontLimit){
          setSpeeds(255, distanceFront*6.3);
        }else if(distanceLeft>=40)
          setSpeeds(255-distanceLeft+25, 255);
        else if(distanceLeft<20){
          setSpeeds(255, 255+distanceLeft-50);
        }
    }else{
      if (distanceFront < frontLimit)
        automatic = true;
    }
  //light
  //Serial.print("Light: ");
  //Serial.println(rate);
  if(rate<800)  
    PORTD = PORTD | (1<<PORTD3);
  else   
    PORTD = PORTD &~(1<<PORTD3);
}


void formatTXAPIpacket(int value) { 

  long sum = 0; // Accumulate the checksum

  // API frame Start Delimiter
  USART_transmit(frameStartByte);

  // Length - High and low parts of the frame length (Number of bytes between the length and the checksum)
  USART_transmit(0x00);
  USART_transmit(0x15);

  // Frame Type - Indicate this frame contains a Transmit Request
  USART_transmit(frameTypeTXrequest); 
  sum += frameTypeTXrequest;

  // Frame ID - set to zero for no reply
  USART_transmit(0x00); 
  sum += 0x00;

  // 64-bit Destination Address - The following 8 bytes indicate the 64 bit address of the recipient (high and low parts).
  // Use 0xFFFF to broadcast to all nodes.
  USART_transmit((destAddressHigh >> 24) & 0xFF);
  USART_transmit((destAddressHigh >> 16) & 0xFF);
  USART_transmit((destAddressHigh >> 8) & 0xFF);
  USART_transmit((destAddressHigh) & 0xFF);
  sum += ((destAddressHigh >> 24) & 0xFF);
  sum += ((destAddressHigh >> 16) & 0xFF);
  sum += ((destAddressHigh >> 8) & 0xFF);
  sum += (destAddressHigh & 0xFF);
  USART_transmit((destAddressLow >> 24) & 0xFF);
  USART_transmit((destAddressLow >> 16) & 0xFF);
  USART_transmit((destAddressLow >> 8) & 0xFF);
  USART_transmit(destAddressLow & 0xFF);
  sum += ((destAddressLow >> 24) & 0xFF);
  sum += ((destAddressLow >> 16) & 0xFF);
  sum += ((destAddressLow >> 8) & 0xFF);
  sum += (destAddressLow & 0xFF);

  // 16-bit Destination Network Address - The following 2 bytes indicate the 16-bit address of the recipient.
  // Use 0xFFFE if the address is unknown.
  USART_transmit(0xFF);
  USART_transmit(0xFE);
  sum += 0xFF+0xFE;

  // Broadcast Radius - when set to 0, the broadcast radius will be set to the maximum hops value
  USART_transmit(0x00);
  sum += 0x00;

  // Options
  // 0x01 - Disable retries and route repair
  // 0x20 - Enable APS encryption (if EE=1)
  // 0x40 - Use the extended transmission timeout
  USART_transmit(0x00);
  sum += 0x00;

  // RF Data
  USART_transmit((value >> 8) & 0xFF);  // ADC temperature reading (high byte)
  USART_transmit(value & 0xFF);  // ADC temperature reading (low byte)
  sum += ((value >> 8) & 0xFF)+(value & 0xFF); 

    // RF Data 1 (Left sensor)
  int dleft=int(distanceLeft);
  USART_transmit((dleft >> 8) & 0xFF);  // left distance reading (high byte)
  USART_transmit(dleft & 0xFF);  // left distance reading (low byte)
  sum += ((dleft >> 8) & 0xFF)+(dleft & 0xFF);
   
  // RF Data 2 (Front sensor)
  int dfront=int(distanceFront);
  USART_transmit((dfront >> 8) & 0xFF);  // left distance reading (high byte)
  USART_transmit(dfront & 0xFF);  // left distance reading (low byte)
  sum += ((dfront >> 8) & 0xFF)+(dfront & 0xFF); 
  
  USART_transmit(rssi); // RSSI reading
  sum += rssi;

  // Checksum = 0xFF - the 8 bit sum of bytes from offset 3 (Frame Type) to this byte.
  USART_transmit( 0xFF - ( sum & 0xFF));

  // Pause to let the microcontroller settle down if needed
  our_delay(7000);
} 

void formatATcommandAPI(char* command) { 
  // Format and transmit a AT Command API frame
  long sum = 0; // Accumulate the checksum

  ATcounter += 1; // increment frame counter

  // API frame Start Delimiter
  USART_transmit(frameStartByte);

  // Length - High and low parts of the frame length (Number of bytes between the length and the checksum)
  USART_transmit(0x00);
  USART_transmit(0x04);

  // Frame Type - Indicate this frame contains a AT Command
  USART_transmit(frameTypeATcommand); 
  sum += frameTypeATcommand;

  // Frame ID – cannot be zero for receiving a reply
  USART_transmit(ATcounter); 
  sum += ATcounter;

  // AT Command
  USART_transmit(command[0]);
  USART_transmit(command[1]);
  sum += command[0];
  sum += command[1];

  // Parameter Value for the Command - Optional

  // Checksum = 0xFF - the 8 bit sum of bytes from offset 3 (Frame Type) to this byte.
  USART_transmit( 0xFF - ( sum & 0xFF));
  
  // Pause to let the microcontroller settle down if needed
  our_delay(7000);
}

//    Right now, PWM output only works on the pins with
// hardware support. 
//    These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWriting_left(uint8_t pin, int val)
{
        // We need to make sure the PWM output is enabled for those pins
        // that support it, as we turn it off when digitally reading or
        // writing with them.  Also, make sure the pin is in output mode
        // for consistenty with Wiring, which doesn't require a pinMode
        // call for the analog output pins.
        
        if (val == 0)
        {
                digitalWrite(pin, LOW);
                //PORTB &= ~(1<<PORTB3);
        }
        else if (val == 255)
        {
                digitalWrite(pin, HIGH);
                //PORTB |= (1<<PORTB3);
        }
        else
        {
                switch(digitalPinToTimer(pin))
                {
                        case TIMER0A:
                                // connect pwm to pin on timer 0, channel A
                                sbi(TCCR0A, COM0A1);
                                //TCCR0A |= 1<<COM0A1;
                                OCR0A = val; // set pwm duty
                                break;

                        case TIMER0B:
                                // connect pwm to pin on timer 0, channel B
                                sbi(TCCR0A, COM0B1);
                                //TCCR0A |= 1<<COM0B1;
                                OCR0B = val; // set pwm duty
                                break;

                        case TIMER1A:
                                // connect pwm to pin on timer 1, channel A
                                sbi(TCCR1A, COM1A1);
                                //TCCR1A |= 1<<COM1A1;
                                OCR1A = val; // set pwm duty
                                break;

                        case TIMER1B:
                                // connect pwm to pin on timer 1, channel B
                                sbi(TCCR1A, COM1B1);
                                //TCCR1A |= 1<<COM1B1;
                                OCR1B = val; // set pwm duty
                                break;

                        case TIMER2A:
                                // connect pwm to pin on timer 2, channel A
                                sbi(TCCR2A, COM2A1);
                                //TCCR2A |= 1<<COM2A1;
                                OCR2A = val; // set pwm duty
                                break;

                        case TIMER2B:
                                // connect pwm to pin on timer 2, channel B
                                //sbi(TCCR2A, COM2B1);
                                TCCR2A |= 1<<COM2B1;
                                OCR2B = val; // set pwm duty
                                break;

                        case NOT_ON_TIMER:
                        default:
                                if (val < 128) {
                                        digitalWrite(pin, LOW);
                                        //PORTB &= ~(1<<PORTB3);
                                } else {
                                        digitalWrite(pin, HIGH);
                                        //PORTB |= (1<<PORTB3);
                                }
                }
        }
}

void analogWriting_right(uint8_t pin, int val)
{
        // We need to make sure the PWM output is enabled for those pins
        // that support it, as we turn it off when digitally reading or
        // writing with them.  Also, make sure the pin is in output mode
        // for consistenty with Wiring, which doesn't require a pinMode
        // call for the analog output pins.
        
        if (val == 0)
        {
                digitalWrite(pin, LOW);
                //PORTD &= ~(1<<PORTD5);
        }
        else if (val == 255)
        {
                digitalWrite(pin, HIGH);
                //PORTD |= (1<<PORTD5);
        }
        else
        {
                switch(digitalPinToTimer(pin))
                {
                        case TIMER0A:
                                // connect pwm to pin on timer 0, channel A
                                sbi(TCCR0A, COM0A1);
                                //TCCR0A |= 1<<COM0A1;
                                OCR0A = val; // set pwm duty
                                break;

                        case TIMER0B:
                                // connect pwm to pin on timer 0, channel B
                                sbi(TCCR0A, COM0B1);
                                //TCCR0A |= 1<<COM0B1;
                                OCR0B = val; // set pwm duty
                                break;

                        case TIMER1A:
                                // connect pwm to pin on timer 1, channel A
                                sbi(TCCR1A, COM1A1);
                                //TCCR1A |= 1<<COM1A1;
                                OCR1A = val; // set pwm duty
                                break;

                        case TIMER1B:
                                // connect pwm to pin on timer 1, channel B
                                sbi(TCCR1A, COM1B1);
                                //TCCR1A |= 1<<COM1B1;
                                OCR1B = val; // set pwm duty
                                break;

                        case TIMER2A:
                                // connect pwm to pin on timer 2, channel A
                                sbi(TCCR2A, COM2A1);
                                //TCCR2A |= 1<<COM2A1;
                                OCR2A = val; // set pwm duty
                                break;

                        case TIMER2B:
                                // connect pwm to pin on timer 2, channel B
                                sbi(TCCR2A, COM2B1);
                                //TCCR2A |= 1<<COM2B1;
                                OCR2B = val; // set pwm duty
                                break;

                        case NOT_ON_TIMER:
                        default:
                                if (val < 128) {
                                        
                                        digitalWrite(pin, LOW);
                                        //PORTD &= ~(1<<PORTD5);
                                } else {
                                        digitalWrite(pin, HIGH);
                                        //PORTD |= (1<<PORTD5);
                                }
                }
        }
}
void our_delay(int x) {

    for(volatile int i = 0; i<=x; i++){}
}


