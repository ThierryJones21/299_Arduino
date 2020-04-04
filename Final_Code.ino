#include <Arduino_LSM6DS3.h> 
#include <WiFiNINA.h>
#include <SPI.h>
#include <String.h>
#define E1 6 //right speed 
#define M1 7 
#define E2 5 //left speed 
#define M2 4 
#define encoder1 2 
#define encoder2 3 
#define desiredSpeed 166 

//Wifi Variables
char ssid[] = "Network";
char pass[] = "Password";

int status = WL_IDLE_STATUS;
IPAddress server(74,125,115,105); 

// Initialize the client library
WiFiClient client;

float deltaTime; 
float revoR = 0; 
float revoL = 0; 
float distR = 0; 
float distL = 0; 
float timeend; 
float veloR; 
float veloL; 
float encoderRightSpeed, encoderLeftSpeed = 166; 
int cntL = 0; 
int cntR = 0; 
float actualLeftSpeed = 0; 
float actualRightSpeed = 0; 
float leftSpeedInc = 0; 
float rightSpeedInc = 0; 
unsigned long initialTime = 0; 
unsigned long currentTime = 0; 
float x, y, z; 
long speedx,speedy,speedz; 
long posx,posy,posz; 
int newTime = 0; 
int xy, finalx, finaly;
int strWiFi[];
  
void setup() { 
  
  pinMode(M1, OUTPUT); 
  pinMode(M2, OUTPUT); 
  pinMode(E1, OUTPUT); 
  pinMode(E2, OUTPUT);
  pinMode(encoder1, INPUT); 
  pinMode(encoder2, INPUT); 

  //attachInterrupt(digitalPinToInterrupt(2), blink, CHANGE); 

  attachInterrupt(2, wheelR, CHANGE); 
  attachInterrupt(3, wheelL, CHANGE); 
  interrupts(); 
  Serial.begin(9600); 
  
  Serial.println("Attempting to connect to WPA network...");
  Serial.print("SSID: ");
  Serial.println(ssid);

  status = WiFi.begin(ssid, pass);
  if ( status != WL_CONNECTED) {
    Serial.println("Couldn't get a wifi connection");
    // don't do anything else after
    while(true);
  }
  else {
    Serial.println("Connected to wifi");
    Serial.println("\nStarting connection...");
    if (client.connect(server, 80)) { // if you get a connection, report back to serial
      Serial.println("connected");
      client.println();
    }
  }  
  }
} 

void wheelR(){ 
  cntR++; 
  revoR = cntR/16.0; 
  distR = revoR*3.25*2*3.1415; 
  timeend = millis()/1000.0; 
  veloR = distR/(deltaTime); 
} 

void wheelL(){ 
  cntL++; 
  revoL = cntL/16.0; 
  distL = revoL*3.25*2*3.1415; 
  timeend = millis()/1000.0; 
  veloL = distL/(deltaTime); 
} 


void loop() { 

  currentTime = millis();   
  deltaTime = currentTime - newTime; 
  
  forward(); //Forward motion start all three robots

  if (IMU.accelerationAvailable()) { 
      IMU.readAcceleration(x, y, z); 
  }    
  // integrate the average accel and add it to the previous speed to calculate the new speed 
  //estimate the average speed since the previous sample, by averaging the two speeds 
  //long newDisplacement = displacement + (avgSpeed * deltaTime); 
  //displacement = newDisplacement; 
   
  long newSpeedX = speedx + (x*9.8  * deltaTime); 
  long newSpeedY = speedy+ (y*9.8 * deltaTime); 
  long newSpeedZ = speedz + (z*9.8  * deltaTime); 
    
  long newPosX = posx + (speedx * deltaTime); 
  long newPosY = posy+ (speedy * deltaTime); 
  long newPosZ = posz + (speedz  * deltaTime); 
  
  
  Serial.println(newSpeedX); 
  Serial.print("\t");  
  Serial.println(newPosX); 
  delay(500); 
  
  leftSpeedInc = 0.5 * round(32*(desiredSpeed - actualLeftSpeed)); 
  encoderLeftSpeed += leftSpeedInc; 

  rightSpeedInc = 0.5 * round(32*(desiredSpeed - actualRightSpeed));  
  encoderRightSpeed += rightSpeedInc; 
                                
  speedx = newSpeedX; 
  speedy = newSpeedY; 
  speedz = newSpeedZ; 
  posx = newPosX; 
  posy = newPosY; 
  posz = newPosZ; 

//########################################################################  
  if (left bumber == hit){//If bumper hits adjust
    adjustRight();
  }
  if (right bumber == hit){//If bumper hits adjust
    adjustLeft();
  }
//If robot finds home base by accelerometer or IR beacon
  if (posz >= 1 || IR beacon is at 0 meters){
      client.println(posx);
      client.println("/");
      client.println(posy);//alert other robots 
      break; 
    }
//If robot gets signal from other robot rotate and drive to location
  if (client.available()){//Go towards robot that found home base
    char data[] = client.read();
    Serial.print(data);
    
    int count = 0;
    int strWiFi = [];
    for (int i = 0; data[i] != '\0'; i++) {
       if (data[i] == ',') { 
              count++;    
       }
       else { 
         strWiFi[count] = strWiFi[count] * 10 + (data[i] - 48); 
          // subtract str[i] by 48 to convert it to int 
          // Generate number by multiplying 10 and adding 
          // (int)(str[i]) 
       }  
    } 
      finalx = (strWiFi[0]); 
      finaly = (strWiFi[1]); 
  
      distancexy = sqrt(pow(finalx,2) + pow(finaly,2)); //Find triangulated distance
     
      rotate(finalx, finaly);//Rotate to position 
      
      driveDistance(distancexy);//Drive to robot
      
      break;//Stop
  }
    
  actualLeftSpeed = ((cntL/16)/((deltaTime)/1000)); 
  actualRightSpeed = ((cntR/16)/((deltaTime)/1000)); 
    
  Serial.print(actualLeftSpeed); 
  Serial.print("\t");  
  Serial.println(actualRightSpeed); 
//##############################################################  
  newTime = currentTime;    //Update time
}                                    
} 

void adjustLeft(){
  digitalWrite(M1, HIGH); 
  digitalWrite(M2, LOW); 
  analogWrite(E1,10); 
  analogWrite(E2,10);
}

void adjustRight(){
  digitalWrite(M1, LOW); 
  digitalWrite(M2, HIGH); 
  analogWrite(E1,10); 
  analogWrite(E2,10);
}

void rotate(int x, int y){
  adjust to location x and y 
  if right{
  digitalWrite(M1, LOW); 
  digitalWrite(M2, HIGH); 
  analogWrite(E1,adjustSpeedx); 
  analogWrite(E2,adjustSpeedy);
  }
  if left{
  digitalWrite(M1, HIGH); 
  digitalWrite(M2, LOW); 
  analogWrite(E1,adjustSpeedx); 
  analogWrite(E2,adjustSpeedy);
  }
}

void driveDistance(int distancexy){
  digitalWrite(M1, LOW); 
  digitalWrite(M2, HIGH); 
  analogWrite(E1,10); 
  analogWrite(E2,10);
}

void forward(){ 
  digitalWrite(M1, HIGH); 
  digitalWrite(M2, HIGH); 
  analogWrite(E1,encoderRightSpeed); 
  analogWrite(E2,encoderLeftSpeed); 

} 
