#define WITHOUT_REGISTER

#ifdef WITHOUT_REGISTER

#define ldr_sensor  A0
#define led 3
#define echo_front 2
#define trigger_front 9
#define echo_left 8
#define trigger_left 10
#define left_motor_enable_pin 5
#define left_motor_1 7
#define left_motor_2 6
#define right_motor_enable_pin 11
#define right_motor_1 12
#define right_motor_2 13

#else

#define ldr_sensor  A0
#define led 3
#define echo_front 2
#define trigger_front 9
#define echo_left 8
#define trigger_left 10
#define left_motor_enable_pin 5
#define left_motor_1 7
#define left_motor_2 6
#define right_motor_enable_pin 11
#define right_motor_1 12
#define right_motor_2 13

#define ldr_sensor  A0
#define D_led DDD3
#define D_echo_front DDD2
#define D_trigger_front DDB1
#define D_echo_left DDB0
#define D_trigger_left DDB2
#define D_left_motor_enable_pin DDD5
#define D_left_motor_1 DDD7
#define D_left_motor_2 DDD6
#define D_right_motor_enable_pin DDB3
#define D_right_motor_1 DDB4
#define D_right_motor_2 DDB5

#define P_led PORTD3
#define P_echo_front PORTD2
#define P_trigger_front PORTB1
#define P_echo_left PORTB0
#define P_trigger_left PORTB2
#define P_left_motor_enable_pin PORTD5
#define P_left_motor_1 PORTD7
#define P_left_motor_2 PORTD6
#define P_right_motor_enable_pin PORTB3
#define P_right_motor_1 PORTB4
#define P_right_motor_2 PORTB5
#endif


#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define BAUD_RATE 9600
#define BAUD_RATE_DIVISOR (F_CPU / 16 / BAUD_RATE - 1)

/*
 * Global variables
 */
unsigned int distanceFront, distanceLeft, frontLimit = 35;
volatile int counter = 0;
const byte frameStartByte = 0x7E;
bool automatic=false, back = false;


/*
 * Auxiliary functions
 */
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
  if(!automatic)  
    {
      if ( ch == 'W')
      {      
        set_speed(255,255);     
      }   
      else if  ( ch == 'S')
      {        
        set_speed(0,0);     
      }
      else if  ( ch == 'A')
      {     
        set_speed(140,255);     
      }
      else if  ( ch == 'D')
      {     

        set_speed(255,140);     
      }
      else if( ch == 'B')
      {
        set_speed(0,0);
        if (!back)
        {
          back = true;
          towardsBack();
        }
        else
        {
          back = false;
          towardsFront();
        }
      }
      else if  ( ch == 'P')
      { 
        automatic=true;     
      }
     }
    else if  ( ch == 'O')
    { 
      automatic=false;   
      set_speed(0,0);   
    }
}

void sendMessage(int value){
  long sum = 0; // Accumulate the checksum

  // API frame Start Delimiter
  USART_transmit(frameStartByte);
  
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
  
  // Checksum = 0xFF - the 8 bit sum of bytes from offset 3 (Frame Type) to this byte.
  USART_transmit( 0xFF - ( sum & 0xFF));

  // Pause to let the microcontroller settle down if needed
  delay(50);  
}

unsigned int get_distance(int trigger, int echo){
  unsigned long duration;
  //clean the trigger pin
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  //send the impulse
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, HIGH);
  return duration * 0.034/2;
}

void set_speed (int left, int right){
  analogWrite(left_motor_enable_pin, left);
  analogWrite(right_motor_enable_pin, right);
}

void towardsFront(){
  #ifdef WITHOUT_REGISTER
  digitalWrite(left_motor_1, HIGH);
  digitalWrite(left_motor_2, LOW);
  digitalWrite(right_motor_1, HIGH);
  digitalWrite(right_motor_2, LOW);
  #else
  PORTD = 1<<P_left_motor_enable_pin | 1<<P_left_motor_1 | 0<<P_left_motor_2;
  PORTB = 1<<P_right_motor_enable_pin | 1<<P_right_motor_1 | 0<<P_right_motor_2;
  #endif
}

void towardsBack(){
  #ifdef WITHOUT_REGISTER
  digitalWrite(left_motor_1, LOW);
  digitalWrite(left_motor_2, HIGH);
  digitalWrite(right_motor_1, LOW);
  digitalWrite(right_motor_2, HIGH);
  #else
  PORTD = 1<<P_left_motor_enable_pin | 0<<P_left_motor_1 | 1<<P_left_motor_2;
  PORTB = 1<<P_right_motor_enable_pin | 0<<P_right_motor_1 | 1<<P_right_motor_2;
  #endif
}
/*
 * Setup function
 */
void setup() {
  USART_init();

  #ifdef WITHOUT_REGISTER
  //Initialize Front Sensor pins
  pinMode(echo_front, INPUT);
  pinMode(trigger_front, OUTPUT);
  //initialize Left sensor pins
  pinMode(echo_left, INPUT);
  pinMode(trigger_left, OUTPUT);
  
  //initialize left motor pins as output 
  pinMode(left_motor_enable_pin, OUTPUT);
  pinMode(left_motor_1, OUTPUT);
  pinMode(left_motor_2, OUTPUT);
  //turn on the left motor with the right direction
  digitalWrite(left_motor_enable_pin, HIGH);
  digitalWrite(left_motor_1, HIGH);
  digitalWrite(left_motor_2, LOW);

  //initialize l motor pins as output 
  pinMode(right_motor_enable_pin, OUTPUT);
  pinMode(right_motor_1, OUTPUT);
  pinMode(right_motor_2, OUTPUT);
  
  //turn on the right motor with the right direction
  digitalWrite(right_motor_enable_pin, HIGH);
  digitalWrite(right_motor_1, HIGH);
  digitalWrite(right_motor_2, LOW);
  
  // initialize led output connected to LDR
  pinMode(led, OUTPUT);

  #else

  //initialize pins
  DDRD = 0<<D_echo_front | 1<<D_left_motor_enable_pin | 1<<D_left_motor_1 | 1<<D_left_motor_2 | 1<<D_led;
  DDRB = 1<<D_trigger_front | 0<<D_echo_left | 1<<D_trigger_left | 1<<D_right_motor_enable_pin | 1<<D_right_motor_1 | 1<<D_right_motor_2;

  //set properly the motors
  PORTD = 1<<P_left_motor_enable_pin | 1<<P_left_motor_1 | 0<<P_left_motor_2;
  PORTB = 1<<P_right_motor_enable_pin | 1<<P_right_motor_1 | 0<<P_right_motor_2;
  
  #endif
}

/*
 * Loop function
 */
void loop() {
  distanceFront = get_distance(trigger_front, echo_front);
  USART_transmit(distanceFront);

  distanceLeft = get_distance(trigger_left, echo_left);

  if(automatic){
    if(distanceFront<frontLimit) //If I am too close
      set_speed(255, distanceFront*3);
    else if(distanceLeft>=30) //If I am too far from the wall I need to go closer
      set_speed(255-distanceLeft+25, 255); // go to the left, decrease left, left right
    else if(distanceLeft<10) //If I am too close to the wall I need to go farther
      set_speed(255, 255+distanceLeft-50); //go to the right, decrease right, increase left
    else
      set_speed(200,200);
  }
  else {
    if (distanceFront < frontLimit)
      automatic = true;
  }
  //measurement of ldr sensor
  unsigned int ldr_value = analogRead(ldr_sensor);
  #ifdef WITHOUT_REGISTER
  
  if (ldr_value < 80)
      digitalWrite(led, HIGH);
  else
      digitalWrite(led, LOW);
      
  #else
  
    if (ldr_value < 80)
      PORTD = PORTD | (1<<P_led);
  else
      PORTD = PORTD &~(1<<P_led);
      
  #endif
  sendMessage(ldr_value);
    
  delay(500);
}
