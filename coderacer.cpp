#include <Wire.h>
#include <SoftwareSerial.h> //Used for transmitting to the device
SoftwareSerial softSerial(12, 13); //RX, TX
#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
#include <string.h>
#include "Person.h"
#include "helper.h"
#define KEY1_PIN 9//
#define KEY2_PIN 8//
#define BUZZER 4
#define BR 115200
RFID nano; //Create instance
void PrintPerson(Person *print , long *laps , int place);
Person * sortlist( Person * head, long *laps);
void Get_Race_Parametes();
void Add_new_competitor();
void Print_competitor(Person *ptr ,int k);
float seconds;
float currentMillis;
float personTime;
unsigned long prevMillis = millis();
bool pause = 1;
String incomingString;//reading name string
String incomingID;//reading ID string
String incomingGender;//reading ID string
int k=0;//for reading loop
    Person* ptr;
    Person *newPerson;
    Person *findmin;
    Person* Head ;
    long Distance;
    long * PDistance;
    long Laps;
    long* PLaps;
  byte myEPC[12]; //Most EPCs are 12 bytes
   byte myEPClength;
   byte responseType = 0;
    int PersonCounter=0; // number of competitors in the race
void setup()
{
  //pin setup
  pinMode(KEY2_PIN, INPUT);
  pinMode(KEY1_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(11, OUTPUT);
  Serial.begin(BR);
  while (!Serial); //Wait for the serial port to come online
  if (setupNano(38400) == false) //Configure nano to run at 38400bps #???#
  {
    Serial.println(F("Module failed to respond. Please check wiring."));
    while (1); //Freeze!
  }

  nano.setRegion(REGION_OPEN);
  nano.setReadPower(2700); //27.00 dBm. Higher values may caues USB port to brown out
  //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling
  Get_Race_Parametes(); // Getting race prametes : Race distance , number of laps

    while(pause == 1)//reading competitors
    {
     Add_new_competitor();
 Serial.println(F("Add a new compettetor ? "));
 Serial.println(F("Yes : press 1 "));
 Serial.println(F("No : press 0 "));
 while (Serial.available()==0){}  // wait for user input
  pause= Serial.parseInt();      
    }

Head = ptr ;
   int k=0;
 while(ptr) //Printing compettetors before starti
    {
  Print_competitor(ptr, k);
 ptr=ptr->next;
 k++;
    } 

  Serial.println(F("Press START key to begin the race"));
  while(!digitalRead(KEY1_PIN));
  digitalWrite(7, HIGH);
  prevMillis = millis();
  Serial.read(); //Throw away the user's character
  nano.startReading(); //Begin scanning for tags
}

void loop()
{
  byte myEPCc[12];
  byte checkepc[12];
  byte myEPCclength;
  myEPCclength = sizeof(myEPCc); 

  if (nano.check() == true) //Check to see if any new data has come in from module
  {
    byte responseType = nano.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

    if (responseType == RESPONSE_IS_KEEPALIVE)
    {
      Serial.println(F("Scanning"));
      
    }
    else if (responseType == RESPONSE_IS_TAGFOUND)
    {
      //If we have a full record we can pull out the fun bits
 //     int rssi = nano.getTagRSSI(); //Get the RSSI for this tag read
  //    long freq = nano.getTagFreq(); //Get the frequency this tag was detected at
   //   long timeStamp = nano.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message
      byte tagEPCBytes = nano.getTagEPCBytes(); //Get the number of bytes of EPC from response
      nano.readTagEPC(myEPCc, myEPCclength, 500); //Scan for a new tag up to 500ms
      
      //Print EPC bytes, this is a subsection of bytes from the response/msg array
      Serial.print(F(" epc["));
      for (byte x = 0 ; x < tagEPCBytes ; x++)
      {
        checkepc[x]= nano.msg[31 + x];
        if (nano.msg[31 + x] < 0x10) Serial.print(F("0")); //Pretty print
        Serial.print(nano.msg[31 + x], HEX);
        Serial.print(F(" "));
      }
      Serial.print(F("]"));
      Serial.println();
      ptr=Head;
      int check ;
      
      while(ptr){ //Check if recived EPC maches the one instance in the linked list 
        check = 0;
        for(int i =0 ; i < tagEPCBytes ;i++)
        {
            if (checkepc[i]==ptr->_personEPC[i])
            check++;
        } 
            if (check == 12){
             if (ptr->lap == 0)  {
               Serial.println(ptr->_personName);
               ptr->_personTime[ptr->lap] = runTimer();
               ptr->lap++;
               digitalWrite(11, HIGH);   // turn the LED on (HIGH is the voltage level)   
               tone(BUZZER, 1000);
               delay(1000);   // wait for a second    
               noTone(BUZZER);
               digitalWrite(11, LOW);    // turn the LED off by making the voltage LOW*/
              }
             else if ((runTimer() - ptr->_personTime[(ptr->lap)-1]) > 5) {
              Serial.println(ptr->_personName);
              ptr->_personTime[ptr->lap] = runTimer();
              ptr->lap++;
              digitalWrite(11, HIGH);   // turn the LED on (HIGH is the voltage level)   
              tone(BUZZER, 1000);
              delay(1000);   // wait for a second    
              noTone(BUZZER);
              digitalWrite(11, LOW);    // turn the LED off by making the voltage LOW*/
              }
            }           
        ptr = ptr->next;
      }
 
      runTimer();
      Serial.println();
     /* digitalWrite(11, HIGH);   // turn the LED on (HIGH is the voltage level)          
       delay(400); 
      digitalWrite(11, LOW);    // turn the LED off by making the voltage LOW*/
    }

  }
  if (digitalRead(KEY2_PIN))
  {
     Serial.println();
     Serial.println("delete");
     Serial.println();
     float  min ;
     ptr = Head ;
     int k =Laps ; 
     while(ptr)
     {
       for(int i =0 ; i < k ; i++)
        {
          if(i==0){
            ptr->_speed_per_lap[i] =( Distance / ptr->_personTime[i])*3.6 ;      
           }
          else {
            ptr->_speed_per_lap[i] = (Distance / (ptr->_personTime[i] - ptr->_personTime[i-1] ))*3.6;      
           }
        }
        ptr->_avg_speed =  (((Distance) * (k)) /ptr->_personTime[k -1]  )*3.6;
      

       ptr=ptr->next;
      }
      ptr = Head ;
      int place =1;

      while(ptr->next){
        findmin = ptr ;
        min = ptr->_personTime[k -1];
        while(findmin){
        if (min > findmin->_personTime[k -1]){
          min = findmin->_personTime[k -1];
        }
          findmin = findmin->next;
        }
         findmin = ptr ;
         while (findmin){
           if (findmin->_personTime[k -1] == min ){
                Serial.println("**********************") ; 
                 PrintPerson(findmin, Laps , place );
                 place ++ ;
                 ptr = findmin->next;
                 findmin = ptr;
                 break ;
           }
           else if (findmin->next->_personTime[k -1] == min ){
            Serial.println("**********************") ; 
             PrintPerson(findmin->next, Laps , place);
                             place ++ ;
             findmin->next=findmin->next->next;
                              break ;

           }
           findmin=findmin->next;

         }
      }
         Serial.println("**********************") ; 
          PrintPerson(ptr, Laps , place);

      
      delay(500);
      exit(0); 
  }
}

//Gracefully handles a reader that is already configured and already reading continuously
//Because Stream does not have a .begin() we have to do this outside the library
boolean setupNano(long baudRate)
{
  nano.begin(softSerial); //Tell the library to communicate over software serial port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while (softSerial.isListening() == false); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while (softSerial.available()) softSerial.read();

  nano.getVersion();

  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano.stopReading();

    Serial.println(F("Module continuously reading. Asking it to stop..."));

    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    softSerial.begin(BR); //Start software serial at 115200

    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    softSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate

    delay(250);
  }

  //Test the connection
  nano.getVersion();
  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right
  //The M6E has these settings no matter what
  nano.setTagProtocol(); //Set protocol to GEN2
  nano.setAntennaPort(); //Set TX/RX antenna ports to 1
  return (true); //We are ready to rock
}

//##########################################################################################################
float runTimer(){
  currentMillis = millis();
  seconds = currentMillis - prevMillis;
  float personTime = seconds/1000;
  //Serial.print(personTime); 
  return personTime;
}
//##########################################################################################################
void PrintPerson(Person *print , long *laps , int place)
{
  int p = laps;
    Serial.println() ; 
  Serial.print("Competitor name : ") ; 
  Serial.println(print->_personName) ; 
   Serial.print("Competitor ID : ") ; 
  Serial.println(print->_personID) ; 
     Serial.print("Competitor gender : ") ; 
  Serial.println(print->gender) ; 
   Serial.print("Competitor place : ") ;
   Serial.println(place) ; 
   Serial.print("Finish time : ") ;
   Serial.println(print->_personTime[p -1]);
   Serial.print("Avrage speed : ") ;
   Serial.println(print->_avg_speed);
   for (int i = 0 ; i < p ; i++ )
   {
    Serial.print("Lap # :") ; 
    Serial.print( i+1 ) ;
    Serial.print("  , Time per lap :  ") ;
    if (i ==0){
    Serial.print(print->_personTime[i]) ;
    Serial.print(" [sec] ") ;
    }
    else {
    Serial.print(print->_personTime[i] - print->_personTime[i-1]) ;
    Serial.print(" [sec] ") ;
    }
    Serial.print("  , Speed per lap :  ") ;
    Serial.print(print->_speed_per_lap[i]) ;
    Serial.println(" [KM/H] ") ;
    Serial.println() ;
   } 
}
//##########################################################################################################
void Get_Race_Parametes()
{
  Serial.println(F("Insert race distance [m]"));
while(!digitalRead(KEY2_PIN));
    Distance= Serial.parseInt();
   // Serial.println(Distance);
        PDistance = &Distance;
    //Serial.println(*PDistance);
Serial.println();

Serial.println(F("Insert number of laps"));
while(!digitalRead(KEY2_PIN));
    Laps= Serial.parseInt();
    PLaps = &Laps;
   // Serial.println(*PLaps);
Serial.println();
}
void Add_new_competitor()
{
Serial.println(F("Add a new competitor"));
  Serial.read();
  newPerson = new Person; //create new person instance 
  Serial.println(F("Please inserts competitor name and press on NEXT key "));
  while(!digitalRead(KEY2_PIN)); //wait for input 
  incomingString = Serial.readString(); // save input 
  newPerson->setName(incomingString); //set saved name to new person instance
  Serial.flush(); //clear buffer 
  Serial.read();
  Serial.println();
  Serial.println(F("Please insert competitor ID and press on NEXT key"));
  while(!digitalRead(KEY2_PIN));
  incomingID = Serial.readString() ; 
  newPerson->setID(incomingID);
    Serial.flush();
    Serial.println();
    Serial.println(F("Please insert competitor Gender [M/F] and press on NEXT key "));
  while(!digitalRead(KEY2_PIN));
  incomingGender = Serial.readString() ; 
  newPerson->gender = incomingGender;
    Serial.flush();  
    Serial.println();
     while (responseType != RESPONSE_SUCCESS)//RESPONSE_IS_TAGFOUND)
  {
    myEPClength = sizeof(myEPC); //Length of EPC is modified each time .readTagEPC is called
    responseType = nano.readTagEPC(myEPC, myEPClength, 500); //Scan for a new tag up to 500ms
    Serial.println(F("Searching for tag"));
    newPerson->setEPC(myEPC);
  }
    responseType=RESPONSE_FAIL;
  if (PersonCounter==0)
  {
    newPerson->next = NULL;
    ptr = newPerson ; 
  } 
  else
  {
   newPerson->next = ptr ; 
   ptr = newPerson ; 
  }
 PersonCounter++;
}
//##########################################################################################################
void Print_competitor(Person *ptr ,int k)
{
        Serial.print("Competitor # ");
       Serial.print(k+1);
        Serial.print(": ");
        Serial.println(ptr->_personName);
        Serial.println(ptr->_personID);
   Serial.print(F(" epc["));
  for (byte x = 0 ; x < myEPClength ; x++)
  {
    if (ptr->_personEPC[x] < 0x10) Serial.print(F("0"));
    Serial.print(ptr->_personEPC[x], HEX);
    Serial.print(F(" "));
  }
  Serial.println(F("]"));
      Serial.println();
}
//##########################################################################################################