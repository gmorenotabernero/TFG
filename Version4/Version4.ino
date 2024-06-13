#include <coap-simple.h>
#include <Arduino.h>
#include <SPI.h>

#include "WiFiEsp.h"
#include <WiFiEspUdp.h>
#include "SoftwareSerial.h"

#define LEDP 13

#define speedPinR 9    //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1  12    //Right Motor direction pin 1 to MODEL-X IN1 
#define RightMotorDirPin2  11    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6    // Left PWM pin connect MODEL-X ENB
#define LeftMotorDirPin1  7    //Left Motor direction pin 1 to MODEL-X IN3 
#define LeftMotorDirPin2  8   //Left Motor direction pin 1 to MODEL-X IN4 


/*motor control*/
void go_Advance(void)  //Forward
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  Serial.println("Hacia delante");
  
}
void go_Left(int t=0)  //Turn left
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  Serial.println("Hacia izquierda");
  delay(t);
}
void go_Right(int t=0)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  Serial.println("Hacia derecha");
  delay(t);
}
void go_Back(int t=0)  //Reverse
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  Serial.println("Hacia atrás");
  delay(t);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
  Serial.println("STOP");
}
/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);
  Serial.println("modifico velocdades");   
}

//Pins initialize
void init_GPIO()
{
  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  
  Serial.println("He inicializado todos los pines como output");
  
  stop_Stop();
}

//Empezamos con el WIFI

 WiFiEspUDP Udp;
 unsigned int localPort = 5683;

 // Los puertos a los que está conectado el ESP
 #define SOFT_RX 4 // RX
 #define SOFT_TX 5 // TX

 SoftwareSerial Serial1(SOFT_RX, SOFT_TX);
int status = WL_IDLE_STATUS;
    //Coap_server.h and ESP8266WiFi.h are
    //The ESP8266 client is connected to the local network by defining the SSID and Password of Wi-Fi router. The SSID and password of Wi-Fi access point are initialized in the Arduino code as follow – 

 char ssid[] = "***************";
 char pass[] = "***************";


// Buffer UDP
 char packetBuffer[5];

 void setup_wifi(){

 Serial1.begin(115200); // Inicializa la comunicación serial for debugging
 Serial1.print("AT+CIOBAUD=9600\r\n");
 Serial1.write("AT+RST\r\n");
 Serial1.begin(9600); // Inicializa la comunicación serial con el módulo ESP

 WiFi.init(&Serial1); // Inicializa el módulo ESP

 if (WiFi.status() == WL_NO_SHIELD) {
 Serial.println("No se ha conectado un WiFi shield!");
 }

 while ( status != WL_CONNECTED) {
 Serial.print("Intentando conectar al SSID: ");
 Serial.println(ssid);
 status = WiFi.begin(ssid, pass);
 }

 IPAddress ip = WiFi.localIP();
 Serial.println("Conectado con la dirección IP: ");
Serial.println(ip);


// Udp.begin(localPort);

 Serial.print("Escuchando en el puerto ");
 Serial.println(localPort);

 }

//Empezamos con el COAP
 

// CoAP server endpoint url callback
void callback_light(CoapPacket &packet, IPAddress ip, int port);

void callback_advance(CoapPacket &packet, IPAddress ip, int port);
void callback_lef(CoapPacket &packet, IPAddress ip, int port);
void callback_right(CoapPacket &packet, IPAddress ip, int port);
void callback_back(CoapPacket &packet, IPAddress ip, int port);

// UDP and CoAP class
 Coap coap(Udp);


bool LEDSTATE;

// CoAP server endpoint URL
void callback_light(CoapPacket &packet, IPAddress ip, int port) {
  Serial.println("[Light] ON/OFF");
  
  // send response
  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;
  
  String message(p);

  Serial.println("Se ha reecibido el payload "+message);

  // En una solicitud PUT, cambia el estado del LED según el valor en el payload
  if (message.equals("0")){
    LEDSTATE = false;
    Serial.println("Apago led");
  }else if(message.equals("1")){
    LEDSTATE = true;
    Serial.println("Enciendo led");
  }
  if (LEDSTATE) {
    digitalWrite(LEDP, HIGH) ; 
    coap.sendResponse(ip, port, packet.messageid, "1");
  } else { 
    digitalWrite(LEDP, LOW) ; 
    coap.sendResponse(ip, port, packet.messageid, "0");
  }
}


void callback_advance(CoapPacket &packet, IPAddress ip, int port) {
  
  // send response
  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;
  
  String message(p);

    go_Advance();    
    coap.sendResponse(ip, port, packet.messageid);
    delay(500);
    stop_Stop();  
}
void callback_left(CoapPacket &packet, IPAddress ip, int port) {
  
  // send response
  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;
  
  String message(p);

    go_Left();    
    coap.sendResponse(ip, port, packet.messageid);
    delay(500);
    stop_Stop();
}
void callback_right(CoapPacket &packet, IPAddress ip, int port) {
  
  // send response
  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;
  
  String message(p);

    go_Right();    
    coap.sendResponse(ip, port, packet.messageid);
    delay(500);
    stop_Stop();
}
void callback_back(CoapPacket &packet, IPAddress ip, int port) {
  
  // send response
  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;
  
  String message(p);

    go_Back();    
    coap.sendResponse(ip, port, packet.messageid);
    delay(500);
    stop_Stop();
}    
void setup() {
  // put your setup code here, to run once:

 Serial.begin(9600);

 init_GPIO();
 setup_wifi();

 
  Serial.println("Setup sll Callbacks");
  
  coap.server(callback_light, "light");
  
  coap.server(callback_advance, "advance");
  coap.server(callback_left, "left");
  coap.server(callback_right, "right");
  coap.server(callback_back, "back");
  

  // start coap server
  coap.start();

}

void loop() {

delay(1000);
  
coap.loop();

}
