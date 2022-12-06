//TODO:: get code working for left and right turns when actual track changes direction
#include <TinyGPS.h>
#include <SoftwareSerial.h>

#define K1 12
#define K2 11
#define K3 8
#define K4 7
#define EN_A 9
#define EN_B 6
#define TRIG 4
#define ECHO 5
#define IR1 52 //White wire
#define IR2 50 //Brown wire
#define IR3 48 //Blue wire
#define IR4 46 //Green wire
#define IR5 44 //Yellow wire
#define countLimitL 100000 //turning left for this amount of time
#define countLimitR 200000 //turning right for double the amout of time as left turn 
#define offTheMapCountLimit 250000 

long distance, Time;
int ir1 = 0;
int ir2 = 0;
int ir3 = 0;
int ir4 = 0;
int ir5 = 0;

//-----GPS-------------

//long lat,lon; //object variables
float lat, lon;
SoftwareSerial gpsSerial(19,18); //rx, tx
TinyGPS gps; // gps object


//---------------------

void MoveForward()
{
  digitalWrite(EN_A, HIGH); //setting EN-A to HIGH
  digitalWrite(K1, HIGH); //K1 motor = HIGH
  digitalWrite(K2, LOW); //K2 motor = LOW
  digitalWrite(K3, HIGH); //K3 motor = HIGH
  digitalWrite(K4, LOW); //K4 motor = LOW
  digitalWrite(EN_B, HIGH); //setting EN-B to HIGH
}

void MoveBackward()
{
  digitalWrite(EN_A, HIGH); //setting EN-A to HIGH
  digitalWrite(K1, LOW); //K1 motor = HIGH
  digitalWrite(K2, HIGH); //K2 motor = LOW
  digitalWrite(K3, LOW); //K3 motor = HIGH
  digitalWrite(K4, HIGH); //K4 motor = LOW
  digitalWrite(EN_B, HIGH); //setting EN-B to HIGH 
}

void Brake()
{
  digitalWrite(EN_A, HIGH); //setting EN-A to HIGH
  digitalWrite(K1, LOW); //K1 motor = HIGH
  digitalWrite(K2, LOW); //K2 motor = LOW
  digitalWrite(K3, LOW); //K3 motor = HIGH
  digitalWrite(K4, LOW); //K4 motor = LOW
  digitalWrite(EN_B, HIGH); //setting EN-B to HIGH
}

void TurnRight() //Front right and Back left move
{
  digitalWrite(EN_A, HIGH); //setting EN-A to HIGH
  digitalWrite(K1, LOW); //K1 motor = HIGH
  digitalWrite(K2, HIGH); //K2 motor = HIGH
  digitalWrite(K3, HIGH); //K3 motor = HIGH
  digitalWrite(K4, LOW); //K4 motor = LOW
  digitalWrite(EN_B, HIGH); //setting EN-B to HIGH
}

void TurnLeft() //Front right and Back left move
{
  digitalWrite(EN_A, HIGH); //setting EN-A to HIGH
  digitalWrite(K1, HIGH); //K1 motor = HIGH
  digitalWrite(K2, LOW); //K2 motor = HIGH
  digitalWrite(K3, LOW); //K3 motor = HIGH
  digitalWrite(K4, HIGH); //K4 motor = LOW
  digitalWrite(EN_B, HIGH); //setting EN-B to HIGH
}

void LineTracker() //fully working, however never tested through function so may have issues with variables since some not global 
{
   ir1 = digitalRead(IR1);  // read the input pin 
   ir2 = digitalRead(IR2);  // read the input pin
   ir3 = digitalRead(IR3);  // read the input pin
   ir4 = digitalRead(IR4);  // read the input pin
   ir5 = digitalRead(IR5);  // read the input pin

  char ir1_char = ir1 + 48; //arduno conversion from int to char is diff from VScode
  String ir1_string = String(ir1_char);
  char ir2_char = ir2 + 48;
  char ir3_char = ir3 + 48;
  char ir4_char = ir4 + 48;
  char ir5_char = ir5 + 48;

  String DigitalReading =  String(ir1_string + ir2_char + ir3_char + ir4_char +ir5_char); //easier to concatenate char into string by using char array
  
  Serial.print("Digital Reading = ");
  Serial.println(DigitalReading);

  if(DigitalReading != "11111")
  {
    MoveForward();
    
      if(DigitalReading == "01111")
      {
        TurnLeft();
      }
      else if(DigitalReading == "11110")
      {
        TurnRight();
      }
      else if(DigitalReading == "11111")
      {
        Brake();
        delay(500);
        
        int counterL = 0;

        counterL = counterL + 1;
        
        while(counterL != countLimitL)
        {
         TurnLeft();
        }

        delay(2000);
        
        if(counterL == countLimitL && DigitalReading == "11111")
          {
            counterL = 0;
            Brake();

            delay(1000);
            
            int counterR = 0;

            counterR = counterR + 1;
            
            while(counterR != countLimitR)
            {
              
            TurnRight();
            
            }

            if(counterR == countLimitR)
            {
              counterR = 0;
            }
            
          }
      }
    }
}

void UltraSonic()
{
  digitalWrite(TRIG, LOW); //have initially off
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH); //send out sonic signal 
  delayMicroseconds(10); //10us period of having it on 

  digitalWrite(TRIG, LOW); 

  //variable that counts how long it takes until it recieves a signal back
  Time = pulseIn(ECHO,HIGH);

  distance = (Time)/58.2; //conversion to CM

  Serial.print("Distance from object: ");
  Serial.println(distance);

   if(distance > 15)
    {
      MoveForward();
    }
    else
    {
      Brake();
      delay(1000);
      MoveBackward();
      delay(500);
    }

  //delay in milliseconds
  delay(50);
}

void setup() 
{
  Serial.begin(9600); //for serial monitor

//------------------------- Pin Attachments --------------------------------------
  pinMode(K1, OUTPUT);// (purple wire)
  pinMode(K2, OUTPUT);// (green wire)
  pinMode(K3, OUTPUT);// (yellow wire)
  pinMode(K4, OUTPUT);// (white wire)

  pinMode(EN_A, OUTPUT); //Enable A (black wire) = pwm signal for K1 and K2 motors
  pinMode(EN_B, OUTPUT); //Enable B (red wire)   = pwm signal for K3 and K4 motors 

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
//--------------------------------------------------------------------------------

Serial.println("The GPS Recieved Signal:");
gpsSerial.begin(9600); // connect to gps sensor
}

void loop()
{
 UltraSonic();

  /*while(gpsSerial.available()) //checking for GPS data
  {
    Serial.println("yup");
    if(gps.encode(gpsSerial.read())) //encode GPS data
    {
      gps.f_get_position(&lat,&lon); //get lattitude and longitude
      Serial.println("Position: ");
      Serial.println("Lattitude: ");
      Serial.println(lat,6);
      Serial.println("Longitude: ");
      Serial.println(lon,6);
    }
  }

  String latitude = String(lat,6);
  String longitude = String(lon, 6);
  //Serial.println(latitude + ";" + longitude);
  delay(1000);*/
  
}
