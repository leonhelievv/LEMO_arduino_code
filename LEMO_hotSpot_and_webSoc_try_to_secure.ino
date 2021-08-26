/*LEMO try to make lemo server secure and trusted
 */
// Import required libraries
#include "SPIFFS.h"
//#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

bool flagSwitchWebsoc = false;
String userCredentials = "";
bool flagDeleteCrdt = false; //test 
bool flagInitWebsoc = false;
bool flagCredentialDialogServed = false;
const int httpsPort = 443; //secure port not (81)

String userSsid = "";
String userPassword = "";

//this is hotSpot
WiFiServer hotSpot(80);
WiFiClient hotSpotClient;

//this is webSoc


//Create AsyncWebServer object on port 81
//AsyncWebServer webSoc(443);
AsyncWebServer webSoc(httpsPort);
//AsyncWebSocket wss("/wss"); try to secure
AsyncWebSocket ws("/ws");


// Variable to store the HTTP request
String header;
//the file that contains html page to request credentials
File dialogHtml;

//S1 port
int long TL_S1Ob = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
int long TL_S1Yb = 0;
int long TL_S1Pb = 0; 
int long TL_S1Bb = 0;
int long TL_S1Op = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
int long TL_S1Yp = 0;
int long TL_S1Pp = 0; 
int long TL_S1Bp = 0;

bool FL_S1Ob = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
bool FL_S1Yb = false;
bool FL_S1Pb = false; 
bool FL_S1Bb = false;
bool FL_S1Op = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
bool FL_S1Yp = false;
bool FL_S1Pp = false; 
bool FL_S1Bp = false;

int  Pw_S1O = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
int  Bd_S1O = 0;
int  Pw_S1Y = 0; 
int  Bd_S1Y = 0;
int  Pw_S1P = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
int  Bd_S1P = 0;
int  Pw_S1B = 0; 
int  Bd_S1B = 0;

bool FL_S1O_on = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
bool FL_S1Y_on = false;
bool FL_S1P_on = false; 
bool FL_S1B_on = false;

//S2 port
int long TL_S2Ob = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
int long TL_S2Yb = 0;
int long TL_S2Pb = 0; 
int long TL_S2Bb = 0;
int long TL_S2Op = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
int long TL_S2Yp = 0;
int long TL_S2Pp = 0; 
int long TL_S2Bp = 0;

bool FL_S2Ob = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
bool FL_S2Yb = false;
bool FL_S2Pb = false; 
bool FL_S2Bb = false;
bool FL_S2Op = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
bool FL_S2Yp = false;
bool FL_S2Pp = false; 
bool FL_S2Bp = false;

int  Pw_S2O = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
int  Bd_S2O = 0;
int  Pw_S2Y = 0; 
int  Bd_S2Y = 0;
int  Pw_S2P = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
int  Bd_S2P = 0;
int  Pw_S2B = 0; 
int  Bd_S2B = 0;

bool FL_S2O_on = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
bool FL_S2Y_on = false;
bool FL_S2P_on = false; 
bool FL_S2B_on = false;
/*
    Carousel pins
    J - Ground
    K - motor power in
    L -
    M - drive input 1
    N - hallEffect 1
    O - (hallEffect 2) eliminated
    P - drive input 2

*/

/* $$  motor driver - parameters and sub routines start $$

    inputs to the driver:
    Flag - A_Flag_AClkw, if true motor turns clock wise
                    if false motor turns Anti Clock wise
    interval_drive int between 1 and 50
     1 is fast, 50 is slow*/

//debug S1 interference with FA and FB
bool FlagTestIrp = false;

String SWver = "LEMO_web_soc_aug16";

/*pin definitions for pico kit*/
int FA_hall1 = 34; //so that we can swap it if wrong wired
//int A_hall2 = 35;
const int A_out1 = 22;
const int A_out2 = 21;

int FB_hall1 = 37;
//int B_hall2 = 33;
const int B_out1 = 23;
const int B_out2 = 19;

const int S1_out1 = 9;//blue wire motor
const int S1_out2 = 10;//pienk wire
const int S1_out3 = 5;//yellow
const int S1_out4 = 18;//orange

const int S2_out1 = 27;//blue
const int S2_out2 = 26;//pienk
const int S2_out3 = 25;//yellow
const int S2_out4 = 0;//orange
/*end of pin definitions*/

int A_interval_drive = 1; //between 1 max and 50 slow
int B_interval_drive = 1; //between 1 max and 50 slow

bool B_Flag_rev_calc = false;
bool B_Flag_started = false;

//+
bool FA_flagDriveDelay = false;
bool FA_Flag_rev_calc = false;
bool FA_Flag_started = false;

//+
bool FB_flagDriveDelay = false;
bool FB_Flag_rev_calc = false;
bool FB_Flag_started = false;

bool Flag_motor_init = false;


/*this indicates turn command is active. example :0/999;*/
bool A_Flag_active_cmnd = false;
int long A_Timer_error_stop;
bool A_Flag_stop_check = false;
bool A_Flag_error_stop = false;
bool A_Flag_wait_stop = false;

bool B_Flag_active_cmnd = false;
int long B_Timer_error_stop;
bool B_Flag_stop_check = false;
bool B_Flag_error_stop = false;
bool B_Flag_wait_stop = false;

int FA_motor_mode = 0;
int const A_state_standing = 0;
int const A_state_start = 1;
int const A_state_running =2;
int const A_state_error_stop = 3;
int const A_state_error_drctn = 4;
int const A_state_parked = 5;
int const A_state_wait_stop = 6;
int const A_state_shake = 7;
int const A_state_crawl = 8;
int const A_state_pendulum = 9;
int const A_state_qrtr_step = 10;
int const A_state_swing_LR = 11;
int const FA_sensor = 12;

//+
int long FA_timerDriveDelay = 0;
int long FB_timerDriveDelay = 0;

int long A_Timer_rev_calc = 0;

int B_motor_mode = 0;
int const B_state_standing = 0;
int const B_state_start = 1;
int const B_state_running = 2;
int const B_state_error_stop = 3;
int const B_state_error_drctn = 4;
int const B_state_parked = 5;
int const B_state_wait_stop = 6;
int const B_state_shake = 7;
int const B_state_crawl = 8;
int const B_state_pendulum = 9;
int const B_state_qrtr_step = 10;
int const B_state_swing_LR = 11;
int const FB_sensor = 12;

int long B_Timer_delayH1_drive = 0;
int long B_Timer_delayH2_drive = 0;
int long B_Timer_rev_calc = 0;

int A_counter_interupts = 0;
int A_count_wrong_direction = 0;

bool A_Flag_drctn_guard = false;
int long A_Timer_err_drctn;

bool FA_qStepGuard = false;
bool FA_qStep_irp = false;
int long FA_qStepTimer;
bool FA_qStepHL = true;
String FA_qStClckw = "";

bool FB_qStepGuard = false;
bool FB_qStep_irp = false;
int long FB_qStepTimer;
bool FB_qStepHL = true;
String FB_qStClckw = "";

int B_counter_interupts = 0;
int B_count_wrong_direction = 0;

bool B_Flag_drctn_guard = false;
int long B_Timer_err_drctn;
bool B_Flag_timer_qrtrd = false;
int long B_Timer_qrtr_drive;

bool B_Flag_delay_qrtrS_lock = false;
int long B_Timer_qrtrS_lock;


/*DRIVER DRIVER DRIVER DRIVER*/
bool FA_Flag_AClkw = true;
int long FA_Timer_Start_pulse;
bool FA_Flag_timer_spulse = false;
int FA_count_start = 4;
bool FA_flipF_start = false;
int FA_pulse_start = 80;
bool FA_Flag_started_error_stop_check = false;
bool FA_drivePhase1 = false;

bool FB_Flag_AClkw = true;
int long B_Timer_Start_pulse;
bool B_Flag_timer_spulse = false;
int B_count_start = 4;
bool B_flipF_start = false;
int B_pulse_start = 80;
bool B_Flag_started_error_stop_check = false;
bool FB_drivePhase1 = false;

String cmnd_full;

/*BUILD COMMAND BUILD COMMAND BUILD COMMAND*/
/*building the command, variables*/

/*++++++ all the WIFI stuff+++++++*/
// Replace with your network credentials
const char* ssid = "SGW";
const char* password = "LeonHelie";

/*HTML and Javascript served when client connects - not used*/
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>LEMO Web Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">  
<title>LEMO Web Server</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:,">
</head>
<body>
  
</body>
</html>
)rawliteral";
 /*____________end of HTML and Java________*/


/*WEB handling routines*/
String notifyClients(String respns) {
  ws.textAll(respns);
};

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;

    String str = (char*)data;
    Serial.println("arrived here >>***> "+str);
    int l = str.length();

    char x = str.charAt(l-1);
    Serial.println(String(x));
   
    if (String(str.charAt(0)) == ":" && String(str.charAt(l-1)) == ";") {
    Serial.println ("valid command" +str);
    notifyClients("valid cmnd >"+str);
    //lauch the cmnd
    cmnd_full=str;
    interpret_command();
    }
  }
};

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      Serial.println('websocket error');
      break;
  }
}


String processor(const String& var){
  Serial.println(var);
  }
/*_________end WEB handeling routines_________*/

bool flgBcmnd;
bool validComnd = false;
String cmndSt;

int current_command;
const int run_motor = 0;
const int stop_motor = 9;
const int shake_motor = 1;
const int crawl_motor = 2;
const int dance_motor = 3;
const int pendulum_motor = 4;
const int qrtr_step_motor = 5;
const int swing_LR_motor = 6;
const int sensor_motor = 7;

String sampleC = ":A0/999;";
char incoming;


bool A_Flag_cmcShake = false;
bool A_Flag_cmc_Crawl = false;
bool A_Flag_qrtr_step = false;
//bool A_Flag_A_swing_timer = false;
bool A_flipFlop = true;

bool B_Flag_cmcShake = false;
bool B_Flag_cmc_Crawl = false;
bool B_Flag_qrtr_step = false;
//bool B_Flag_B_swing_timer = false;
bool B_flipFlop = true;

unsigned long A_Timer_cmcCrawl;
unsigned long A_Timer_cmcShake;
const long Delay_cmcCrawl = 5000;//delay between step commands
const long Delay_cmcShake = 40; //delay between pulses sent to motor

unsigned long B_Timer_cmcCrawl;
unsigned long B_Timer_cmcShake;

bool A_Flag_carouselStanding = true;

String respond_to_App;
bool B_Flag_carouselStanding = true;

unsigned long A_Timer_wait_for_stop;
bool A_Flag_wait_for_stop = false;

unsigned long B_Timer_wait_for_stop;
bool B_Flag_wait_for_stop = false;

bool A_Flag_shake_in_progres = false;
bool A_Flag_crawl_in_progres = false;
bool A_Flag_pendulum_in_progres = false;
bool A_Flag_qrtr_step_in_progres = false;

bool B_Flag_shake_in_progres = false;
bool B_Flag_crawl_in_progres = false;
bool B_Flag_pendulum_in_progres = false;
bool B_Flag_qrtr_step_in_progres = false;

int long A_time_pendlm;
bool A_Flag_timer_pndlm = false;
bool A_Flag_pendulum_active = false;

int long B_time_pendlm;
bool B_Flag_timer_pndlm = false;
bool B_Flag_pendulum_active = false;

int A_Flag_pulse_irp = false;
int long A_loop_delay;

int  A_pendulum_time = 0;

int B_Flag_pulse_irp = false;
int long B_loop_delay;

int  B_pendulum_time = 0;

//swing
int FA_swing_phase = 1;
bool FA_swingDriveA = true;
int FB_swing_phase = 1;
bool FB_swingDriveA = true;

bool A_Flag_swing_in_progres;
bool B_Flag_swing_in_progres;

//sensor
bool A_Flag_sensor;
bool B_Flag_sensor;

//----stepper1------
int delay_stepper1_phase;
int long Timer_stepper1_phase;
//int number_of_steps1 = 400;
bool Flag_stepper1_clckw = true;

int step1_digrees = 360;
int stepper1_count_steps;
int stepper1_step_phase;

bool Flag_new_stepper1_command = false;
bool Flag_stepper1_busy = false;
bool Flag_timer_stepper1_phase = false;

//----stepper2------
int delay_stepper2_phase;
int long Timer_stepper2_phase;
//int number_of_steps2 = 400;
bool Flag_stepper2_clckw = true;

int step2_digrees = 360;
int stepper2_count_steps;
int stepper2_step_phase;

bool Flag_new_stepper2_command = false;
bool Flag_stepper2_busy = false;
bool Flag_timer_stepper2_phase = false;

//comander ID
String FA_cmdr_id = "none";
String FB_cmdr_id = "none";
String S1_cmdr_id = "none";
String S2_cmdr_id = "none";
String SB1R_cmdr_id = "none";
String SB1F_cmdr_id = "none";
String SB2R_cmdr_id = "none";
String SB2F_cmdr_id = "none";       
/*END END END BUILD COMMAND END END END*/

//test timer
bool Flag_test_timer = true;
int long Timer_test_time = millis()+4000;
int long loop_time;
int loop_time_store = 0;

//sensor triger buffer
String SB1R = "ssssssss";//":S1/Sensor wipe/201/700/100/;";
String SB1F = "ssssssss";
String SB2R = "ssssssss";//":S1/Sensor wipe/201/700/100/;";
String SB2F = "ssssssss";

bool BT_reset = false;

void setup() {
  Serial.begin(115200); //Start Serial monitor
    if (SPIFFS.begin(true)) {
    
        //testing dele
        if(flagDeleteCrdt == true){
          SPIFFS.remove("/wifiCrdt.txt");
         }
     
   //SPIFFS opened ok
   Serial.println("SPIFFS ok");
   //is there a credentials file ?
    const bool isCrdt = SPIFFS.exists("/wifiCrdt.txt");
    Serial.println("wifiCrdt.txt exist:  "+String(isCrdt)); 
    if(isCrdt == true){
      //there is a cxredentials file    
      //get user credentials
      bool validCrd = getCredentials();
      if(validCrd){
        Serial.println("flag to init as webSoc  "+userCredentials);
        flagInitWebsoc = true;
      }else{
        Serial.println("fail - file does not contain credentials");
      }             
    }else{
      //there is no credentials file
      Serial.println("flag to init as hotSpot");
      flagInitWebsoc = false;
   }
  
  //deside what LEMO should be
  if(flagInitWebsoc == true){
     Serial.println("init as webSoc with credentials");
     initWebsoc(); 
  }else{
    //init as hotSpot
    Serial.println("init as hotSpot to get credentials");
    initHotspot();
  } 
  }else{
   //SPIFFS fail
   Serial.println("SPIFFS fail");
  }

//-------------------------
//Serial.println("hall1 irp  "+String (loop_time_store));
  pinMode(FA_hall1, INPUT);
//x  pinMode(A_hall2, INPUT);

  pinMode(A_out1, OUTPUT);
  pinMode(A_out2, OUTPUT);

  pinMode(FB_hall1, INPUT);
//x  pinMode(B_hall2, INPUT);

  pinMode(B_out1, OUTPUT);
  pinMode(B_out2, OUTPUT);

  pinMode(S1_out1, OUTPUT);
  pinMode(S1_out2, OUTPUT);
  pinMode(S1_out3, OUTPUT);
  pinMode(S1_out4, OUTPUT);

  pinMode(S2_out1, OUTPUT);
  pinMode(S2_out2, OUTPUT);
  pinMode(S2_out3, OUTPUT);
  pinMode(S2_out4, OUTPUT);

  //set stepper1 in rest
  digitalWrite(S1_out1, LOW);
  digitalWrite(S1_out2, LOW);
  digitalWrite(S1_out3, LOW);
  digitalWrite(S1_out4, LOW);

  //set stepper2 in rest
  digitalWrite(S2_out1, LOW);
  digitalWrite(S2_out2, LOW);
  digitalWrite(S2_out3, LOW);
  digitalWrite(S2_out4, LOW);

};//end of setup


void A_do_LH_out() {
  digitalWrite(A_out1, LOW);//clkw
  digitalWrite(A_out2, HIGH);//clkw
  FA_Flag_timer_spulse = true;
  FA_Timer_Start_pulse = millis() + FA_pulse_start;
};

void B_do_LH_out() {
  digitalWrite(B_out1, LOW);//clkw
  digitalWrite(B_out2, HIGH);//clkw
  B_Flag_timer_spulse = true;
  B_Timer_Start_pulse = millis() + B_pulse_start;
};

void A_do_HL_out() {
  digitalWrite(A_out1, HIGH);//aClkw
  digitalWrite(A_out2, LOW);//aClkw
  FA_Flag_timer_spulse = true;
  FA_Timer_Start_pulse = millis() + FA_pulse_start;
};

void B_do_HL_out() {
  digitalWrite(B_out1, HIGH);//aClkw
  digitalWrite(B_out2, LOW);//aClkw
  B_Flag_timer_spulse = true;
  B_Timer_Start_pulse = millis() + B_pulse_start;
};

//<<<<<<<< itrp (first) >>>>>>>>>>>

void A_startIrp() {
  A_counter_interupts++;
};

void B_startIrp() {
  B_counter_interupts++;
};


//+ the new motor driver
/*
1) called from FA_drive(), timer expired to get here and the interrupt ocures
*/
void FA_h1intrpRising() {
  //Serial.println("---at interupt rise ---");
  //drive
  A_counter_interupts++;
  detachInterrupt(FA_hall1);
  digitalWrite(A_out1, LOW);
  digitalWrite(A_out2, HIGH);
  //restart timer
  FA_timerDriveDelay = millis() + A_interval_drive;
  FA_flagDriveDelay = true;
  
//Serial.println("h1 is HIGH"); 
};

void FA_h1intrpFalling() {
  //Serial.println("---at interupt fall ---");
  //drive
  detachInterrupt(FA_hall1); 
  digitalWrite(A_out1, HIGH);
  digitalWrite(A_out2, LOW);
  //restart timer
  FA_timerDriveDelay = millis() + A_interval_drive;
  FA_flagDriveDelay = true;
      
//Serial.println("h1 is LOW"); 
};

void FB_h1intrpRising() {
  //Serial.println("---at interupt rise ---");
  //drive
  B_counter_interupts++;
  detachInterrupt(FB_hall1);
  digitalWrite(B_out1, LOW);
  digitalWrite(B_out2, HIGH);
  //restart timer
  FB_timerDriveDelay = millis() + B_interval_drive;
  FB_flagDriveDelay = true;
  
//Serial.println("h1 is HIGH"); 
};

void FB_h1intrpFalling() {
  //Serial.println("---at interupt fall ---");
  //drive
  detachInterrupt(FB_hall1); 
  digitalWrite(B_out1, HIGH);
  digitalWrite(B_out2, LOW);
  //restart timer
  FB_timerDriveDelay = millis() + B_interval_drive;
  FB_flagDriveDelay = true;
      
//Serial.println("h1 is LOW"); 
};

void A_sensor_h1_intrp_rise() {
  //implement sensor buffer
//  Serial.println("A_hall1 sensor interupt rise, cmnd in buffer "+SB1R);
//  Serial.println("A_motor_mode "+String(FA_motor_mode));
  if (FA_motor_mode == FA_sensor) {
  /* read the command stored in the sensor buffer and call
  interpreter this cmnd was loaded when the triger was set*/
  //detach interupt prevent further trigers
  detachInterrupt(FA_hall1);//testing
  cmnd_full = SB1R;
  interpret_command();
    //send_respons_to_App 
    String Asns = ":FA/"+FA_cmdr_id+"/sensor/"+String(digitalRead(FA_hall1))+"/;";
    send_respons_to_App(Asns);  
  }
};

void A_sensor_h1_intrp_fall() {
  //implement sensor buffer
//  Serial.println("A_hall1 sensor interupt fall, cmnd in buffer "+SB1F);
  if (FA_motor_mode == FA_sensor) {
  /* read the command stored in the sensor buffer and call
  interpreter this cmnd was loaded when the triger was set*/
  //detach interupt prevent further trigers
  detachInterrupt(FA_hall1);//testing
  cmnd_full = SB1F;
  interpret_command();
    //send_respons_to_App 
    String Asns = ":FA/"+FA_cmdr_id+"/sensor/"+String(digitalRead(FA_hall1))+"/;";
    send_respons_to_App(Asns);  
  }
};

void B_sensor_h1_intrp_rise() {
  //implement sensor buffer
//  Serial.println("FB_hall1 sensor interupt, cmnd in buffer "+SB2R);
  if (B_motor_mode == FB_sensor) {
  /* read the command stored in the sensor buffer and call
  interpreter this cmnd was loaded when the triger was set*/
  //detach interupt prevent further trigers
  detachInterrupt(FB_hall1);//testing
  cmnd_full = SB2R;
  interpret_command();
    
    //send_respons_to_App 
    String Asns = ":FB/"+FB_cmdr_id+"/sensor/"+String(digitalRead(FB_hall1))+"/;";
    send_respons_to_App(Asns);  
  }
};

void B_sensor_h1_intrp_fall() {
  //implement sensor buffer
//  Serial.println("FB_hall1 sensor interupt, cmnd in buffer "+SB2F);
  if (B_motor_mode == FB_sensor) {
  /* read the command stored in the sensor buffer and call
  interpreter this cmnd was loaded when the triger was set*/
  //detach interupt prevent further trigers
  detachInterrupt(FB_hall1);//testing
  cmnd_full = SB2F;
  interpret_command();
    
    //send_respons_to_App 
    String Asns = ":FB/"+FB_cmdr_id+"/sensor/"+String(digitalRead(FB_hall1))+"/;";
    send_respons_to_App(Asns);  
  }
};


void A_motor_reset() {

  //reset all and wait for timer to expier 
  //Serial.println("^^^ arrived at motor_reset ^^^");
  detachInterrupt(FA_hall1);

  FA_flagDriveDelay = false;
  FA_Flag_started = false;
  FA_timerDriveDelay = 0;
  
  //reset other timers
//sx  A_Flag_A_swing_timer = false;
  A_Flag_cmcShake = false;
  A_Flag_cmc_Crawl = false;
  A_Flag_qrtr_step = false;
  A_Flag_shake_in_progres = false;
  A_Flag_crawl_in_progres = false;
  A_Flag_pendulum_in_progres = false;
  A_Flag_timer_pndlm = false;
//  A_Flag_timer_qrtrd = false;
  FA_qStepGuard = false;
//  A_Flag_delay_qrtrS_lock = false;
  A_Flag_wait_for_stop = false;
};

void B_motor_reset() {

  //reset all and wait for timer to expier
  //Serial.println("^^^ arrived at motor_reset ^^^");
  detachInterrupt(FB_hall1);
  
//x  detachInterrupt(B_hall2);
//  B_Flag_delayH1_drive = false;
//  B_Flag_delayH2_drive = false;

//+
  FB_flagDriveDelay = false;
  FB_Flag_started = false;
  B_Flag_started = false;
  
//x  B_Timer_delayH1_drive = 0;
//  B_Timer_delayH2_drive = 0;
  
  //reset other timers
//sx  B_Flag_B_swing_timer = false;
  B_Flag_cmcShake = false;
  B_Flag_cmc_Crawl = false;
  B_Flag_qrtr_step = false;
  B_Flag_shake_in_progres = false;
  B_Flag_crawl_in_progres = false;
  B_Flag_pendulum_in_progres = false;
  B_Flag_timer_pndlm = false;
  B_Flag_timer_qrtrd = false;
  B_Flag_delay_qrtrS_lock = false;
  B_Flag_wait_for_stop = false;
};


//<<<<<<<<<<<  start >>>>>>>>>>
void A_do_start() {
//x
//mod new driver
  
  //Serial.println("=== arrive at motor_start ===");
  attachInterrupt(FA_hall1, A_startIrp, RISING);
  if (FA_count_start != 0) {
    FA_count_start --;
    if (FA_flipF_start == true) {
      FA_flipF_start = false;
      if (FA_Flag_AClkw == true) {
        A_do_LH_out();
      } else {
        A_do_HL_out();
      }
    } else {
      FA_flipF_start = true;
      if (FA_Flag_AClkw == true) {
        A_do_HL_out();
      } else {
        A_do_LH_out();
      }
    }
  } else {
    FA_Flag_started = true;   
    detachInterrupt(FA_hall1);
    
//may be this is where the motor is moveing and running can start
    // test make the timer active
  //FA_flagDriveDelay = true;  
  FA_drive(); 
    
  }
};//end of start

void B_do_start() {
  //Serial.println("=== arrive at motor_start ===");
  attachInterrupt(FB_hall1, B_startIrp, RISING);
  if (B_count_start != 0) {
    B_count_start --;
    if (B_flipF_start == true) {
      B_flipF_start = false;
      if (FB_Flag_AClkw == true) {
        B_do_LH_out();
      } else {
        B_do_HL_out();
      }
    } else {
      B_flipF_start = true;
      if (FB_Flag_AClkw == true) {
        B_do_HL_out();
      } else {
        B_do_LH_out();
      }
    }
  } else {
    B_Flag_started = true;
    detachInterrupt(FB_hall1);
    
  FB_drive(); 
    
  }
};//end of start


void A_Timer_rev_count() {
  if (FA_Flag_rev_calc == true) {
    //Serial.println("Flag_rev_calc " + String(Flag_rev_calc));
    if (millis() >= A_Timer_rev_calc) {
      //Flag_rev_calc = false;
      //calc revs
      //Serial.println("A_intrp per second : "+String(A_counter_interupts));
      A_counter_interupts = 0;
      A_Timer_rev_calc = millis() + 1000;
      //Flag_rev_calc = true;
    }
  }
};

void B_Timer_rev_count() {
  if (B_Flag_rev_calc == true) {
    //Serial.println("Flag_rev_calc " + String(Flag_rev_calc));
    if (millis() >= B_Timer_rev_calc) {
      //Flag_rev_calc = false;
      //calc revs
      //Serial.println("B_intrp per second : "+String(B_counter_interupts));
      B_counter_interupts = 0;
      B_Timer_rev_calc = millis() + 1000;
      //Flag_rev_calc = true;
    }
  }
};

void A_do_error_stop_check() {
  //if rev count is 0 start timer if remains 0 for 1 second flag error stop
  if (A_counter_interupts == 0) {
    //if already running test for expiary
    //Serial.println("stopped !!!");
    if (A_Flag_stop_check == true) {
      if (millis() >= A_Timer_error_stop) {
        //rev's has been 0 for more that 1 second
        A_Flag_error_stop = true;
        FA_motor_mode = A_state_error_stop;
//        Serial.println("error standing");
      }
    } else { //not already running start timer
      A_Timer_error_stop = millis() + 1000;
      A_Flag_stop_check = true;
    }

  } else {
    //reset timer
    A_Timer_error_stop = millis() + 1000;
  }
};

void B_do_error_stop_check() {
  //if rev count is 0 start timer if remains 0 for 1 second flag error stop
  if (B_counter_interupts == 0) {
    //if already running test for expiary
    //Serial.println("stopped !!!");
    if (B_Flag_stop_check == true) {
      if (millis() >= B_Timer_error_stop) {
        //rev's has been 0 for more that 1 second
        B_Flag_error_stop = true;
        B_motor_mode = B_state_error_stop;
//        Serial.println("error standing");
      }
    } else { //not already running start timer
      B_Timer_error_stop = millis() + 1000;
      B_Flag_stop_check = true;
    }

  } else {
    //reset timer
    B_Timer_error_stop = millis() + 1000;
  }
};

void A_do_shake() {
  //Serial.println("arrived at shake");
  //reset the motor
  A_motor_reset();
  //start the shake timer
  A_Flag_cmcShake = true;
  A_flipFlop = true;
  A_Timer_cmcShake = millis() + Delay_cmcShake;
  //report shake started
  String Ask = ":FA/"+FA_cmdr_id+"/"+"shake"+"/;";
  send_respons_to_App(Ask);
};

void B_do_shake() {
  //Serial.println("arrived at shake");
  //reset the motor
  B_motor_reset();
  //start the shake timer
  B_Flag_cmcShake = true;
  B_flipFlop = true;
  B_Timer_cmcShake = millis() + Delay_cmcShake;
  //report shake started
  String Bsk = ":BM/"+FB_cmdr_id+"/"+"shake"+"/;";
  send_respons_to_App(Bsk);
};

void A_do_crawl() {
  //Serial.println("arrived at crawl");
  //reset the motor
  A_motor_reset();
  current_command = crawl_motor;
  A_Flag_active_cmnd = true;
  //start the crawl timer
  A_Flag_cmc_Crawl = true;
  //Serial.println("A_Flag_cmc_Crawl = true : result : "+String(A_Flag_cmc_Crawl));
  A_flipFlop = true;
  A_Timer_cmcCrawl = millis() + Delay_cmcCrawl;
  //report crawl started
  String Ac = ":AM/"+FA_cmdr_id+"/"+"crawl"+"/;";
  send_respons_to_App(Ac);
};

void B_do_crawl() {
  //Serial.println("arrived at crawl");
  //reset the motor
  B_motor_reset();
  current_command = crawl_motor;
  B_Flag_active_cmnd = true;
  //start the crawl timer
  B_Flag_cmc_Crawl = true;
  //Serial.println("B_Flag_cmc_Crawl = true : result : "+String(B_Flag_cmc_Crawl));
  B_flipFlop = true;
  B_Timer_cmcCrawl = millis() + Delay_cmcCrawl;
  //report crawl started
  String Bc = ":BM/"+FB_cmdr_id+"/"+"crawl"+"/;";
  send_respons_to_App(Bc);
};

void A_do_pendulum() {
  //Serial.println("arrived at pendulum");
  if (A_Flag_pendulum_active == true){
    A_Flag_pendulum_active = false;
    digitalWrite(A_out1, HIGH);
    digitalWrite(A_out2, HIGH);
    A_time_pendlm = millis()+A_pendulum_time;
    A_Flag_timer_pndlm = true;
  }else{
    A_Flag_pendulum_active = true;
    digitalWrite(A_out1, LOW);
    digitalWrite(A_out2, HIGH);
    A_time_pendlm = millis()+80;
    A_Flag_timer_pndlm = true;
  }
};

void B_do_pendulum() {
  //Serial.println("arrived at pendulum");
  if (B_Flag_pendulum_active == true){
    B_Flag_pendulum_active = false;
    digitalWrite(B_out1, HIGH);
    digitalWrite(B_out2, HIGH);
    B_time_pendlm = millis()+B_pendulum_time;
    B_Flag_timer_pndlm = true;
  }else{
    B_Flag_pendulum_active = true;
    digitalWrite(B_out1, LOW);
    digitalWrite(B_out2, HIGH);
    B_time_pendlm = millis()+80;
    B_Flag_timer_pndlm = true;
  }
};

void FA_Qstep() {
Serial.println("arrived at new qStep");
  current_command = qrtr_step_motor;
  A_Flag_active_cmnd = true;

  //new starts here
  FA_qStep_irp = false;
  //set timer A_Timer_qrtr_drive
  FA_qStepGuard = true;
  FA_qStepTimer = millis() + 100;
  if (FA_qStClckw == "1"){
    attachInterrupt(FA_hall1, FA_qStIrp, RISING);
    Serial.println("attached RISING");
  }else{
    attachInterrupt(FA_hall1, FA_qStIrp, FALLING);
    Serial.println("attached FALLING");
  }; 
  FA_qStepHL = true;
  digitalWrite(A_out1, HIGH);
  digitalWrite(A_out2, LOW);  
};

void FA_qStIrp(){
  FA_qStepGuard = false;   
  detachInterrupt(FA_hall1);
  FA_qStepTimer = millis() + 1000;
  FA_qStepGuard = true;
  FA_qStep_irp = true;
  FA_qStepHL = false;
  Serial.println("arrived at IRP of new qStep");
  
}//end of FA_qStepIrp

void FB_Qstep() {
Serial.println("arrived at new qStep");
  current_command = qrtr_step_motor;
  B_Flag_active_cmnd = true;

  //new starts here
  FB_qStep_irp = false;
  //set timer A_Timer_qrtr_drive
  FB_qStepGuard = true;
  FB_qStepTimer = millis() + 100;
  if (FB_qStClckw == "1"){
    attachInterrupt(FB_hall1, FB_qStIrp, RISING);
    Serial.println("attached RISING");
  }else{
    attachInterrupt(FB_hall1, FB_qStIrp, FALLING);
    Serial.println("attached FALLING");
  }; 
  FB_qStepHL = true;
  digitalWrite(B_out1, HIGH);
  digitalWrite(B_out2, LOW);  
};

void FB_qStIrp(){
  FB_qStepGuard = false;   
  detachInterrupt(FB_hall1);
  FB_qStepTimer = millis() + 1000;
  FB_qStepGuard = true;
  FB_qStep_irp = true;
  FB_qStepHL = false;
  Serial.println("arrived at IRP of new FB qStep");
  
}//end of FB_qStepIrp
/*A swing routines mod*/

void A_swing_Aclkw(){
//  Serial.println("FA @@ swing Aclkw");
  FA_swing_phase = 1;
  attachInterrupt(FA_hall1,FA_IRP_Swing,RISING);
  //kick start
  if (digitalRead(FA_hall1) == LOW){
    FA_swingDriveA = true;
    digitalWrite(A_out1,HIGH);
    digitalWrite(A_out2,LOW);
  }else{
    FA_swingDriveA = false;
    digitalWrite(A_out1,LOW);
    digitalWrite(A_out2,HIGH);
  }
};

void A_swing_Clkw(){
 Serial.println("h1 is before swing >>  "+String(digitalRead(FA_hall1)));
//  Serial.println("FA @@ swing clkw");  
  attachInterrupt(FA_hall1,FA_IRP_Swing,FALLING);
  //kick start
  if (digitalRead(FA_hall1) == LOW){
    FA_swingDriveA = false;
    digitalWrite(A_out1,LOW);
    digitalWrite(A_out2,HIGH);
  }else{
    FA_swingDriveA = true;
    digitalWrite(A_out1,HIGH);
    digitalWrite(A_out2,LOW);
  }
};

void FA_IRP_Swing(){
  if (FA_swing_phase == 1){
    FA_swing_phase = 2;
    //drive
    if (FA_swingDriveA == true){
      digitalWrite(A_out1,LOW);
      digitalWrite(A_out2,HIGH);
    }else{
      digitalWrite(A_out1,HIGH);
      digitalWrite(A_out2,LOW);
    };
    
  }else if (FA_swing_phase == 2){
    FA_swing_phase = 3;
    if (FA_swingDriveA == true){
      digitalWrite(A_out1,HIGH);
      digitalWrite(A_out2,LOW);
    }else{
      digitalWrite(A_out1,LOW);
      digitalWrite(A_out2,HIGH);
    };
       
  }else{
     detachInterrupt(FA_hall1);
     //rest
     digitalWrite(A_out1,LOW);
     digitalWrite(A_out2,LOW); 
     //Serial.println("h1 is >>  "+String(digitalRead(FA_hall1)));
    }
};

/*B swing routines*/
void B_swing_Aclkw(){
//  Serial.println("FB @@ swing Aclkw");
  FB_swing_phase = 1;
  attachInterrupt(FB_hall1,FB_IRP_Swing,RISING);
  //kick start
  if (digitalRead(FB_hall1) == LOW){
    FB_swingDriveA = true;
    digitalWrite(B_out1,HIGH);
    digitalWrite(B_out2,LOW);
  }else{
    FA_swingDriveA = false;
    digitalWrite(B_out1,LOW);
    digitalWrite(B_out2,HIGH);
  }
};

void B_swing_Clkw(){
 Serial.println("h1 is before swing >>  "+String(digitalRead(FB_hall1)));
//  Serial.println("FB @@ swing clkw");  
  attachInterrupt(FB_hall1,FB_IRP_Swing,FALLING);
  //kick start
  if (digitalRead(FB_hall1) == LOW){
    FB_swingDriveA = false;
    digitalWrite(B_out1,LOW);
    digitalWrite(B_out2,HIGH);
  }else{
    FB_swingDriveA = true;
    digitalWrite(B_out1,HIGH);
    digitalWrite(B_out2,LOW);
  }
};

void FB_IRP_Swing(){
  if (FB_swing_phase == 1){
    FB_swing_phase = 2;
    //drive
    if (FB_swingDriveA == true){
      digitalWrite(B_out1,LOW);
      digitalWrite(B_out2,HIGH);
    }else{
      digitalWrite(B_out1,HIGH);
      digitalWrite(B_out2,LOW);
    };
    
  }else if (FB_swing_phase == 2){
    FB_swing_phase = 3;
    if (FB_swingDriveA == true){
      digitalWrite(B_out1,HIGH);
      digitalWrite(B_out2,LOW);
    }else{
      digitalWrite(B_out1,LOW);
      digitalWrite(B_out2,HIGH);
    };
       
  }else{
     detachInterrupt(FB_hall1);
     //rest
     digitalWrite(B_out1,LOW);
     digitalWrite(B_out2,LOW); 
     //Serial.println("h1 is >>  "+String(digitalRead(FB_hall1)));
    }
};


/* stepper routines */
void do_stepper1(){
  Flag_stepper1_busy = true;
  Flag_new_stepper1_command = false;
  stepper1_step_phase = 1;
  //translate step_digrees into number of steps
  stepper1_count_steps = step1_digrees;
  //Serial.println("Stepper1_digrees : "+String(step1_digrees));
  //set delay (between phase n and n+1)timer to start stepping
  Timer_stepper1_phase = micros()+delay_stepper1_phase;
  //rase the flag to make timer active
  Flag_timer_stepper1_phase = true;
};

void do_stepper2(){
  //Serial.println("arrived at do_stepper2()");
  Flag_stepper2_busy = true;
  Flag_new_stepper2_command = false;
  stepper2_step_phase = 1;
  //translate step_digrees into number of steps
  stepper2_count_steps = step2_digrees;
  //set delay (between phase n and n+1)timer to start stepping
  Timer_stepper2_phase = micros()+delay_stepper2_phase;
  //rase the flag to make timer active
  Flag_timer_stepper2_phase = true;
};

void do_stepper1_step(){//called when step_phase timer triger
int S1_i;
if (Flag_stepper1_clckw != true){
  S1_i = stepper1_step_phase;
  }else{
  S1_i = 8 - stepper1_step_phase+1;
  }
  
  switch (S1_i) {
   case 1:    
   digitalWrite(S1_out1, HIGH);// 1
   digitalWrite(S1_out2, LOW);
   digitalWrite(S1_out3, LOW);
   digitalWrite(S1_out4, LOW);
   break;
   case 2: 
  digitalWrite(S1_out1, HIGH);// 2 
  digitalWrite(S1_out2, HIGH);
  digitalWrite(S1_out3, LOW);
  digitalWrite(S1_out4, LOW);
  break;
   case 3: 
  digitalWrite(S1_out1, LOW);// 3 
  digitalWrite(S1_out2, HIGH);
  digitalWrite(S1_out3, LOW);
  digitalWrite(S1_out4, LOW);
   break;
   case 4: 
  digitalWrite(S1_out1, LOW);// 4 
  digitalWrite(S1_out2, HIGH);
  digitalWrite(S1_out3, HIGH);
  digitalWrite(S1_out4, LOW);
   break;
   case 5: 
  digitalWrite(S1_out1, LOW);// 5
  digitalWrite(S1_out2, LOW);
  digitalWrite(S1_out3, HIGH);
  digitalWrite(S1_out4, LOW);
   break;
   case 6: 
  digitalWrite(S1_out1, LOW);// 6
  digitalWrite(S1_out2, LOW);
  digitalWrite(S1_out3, HIGH);
  digitalWrite(S1_out4, HIGH);
   break;
   case 7: 
  digitalWrite(S1_out1, LOW);// 7
  digitalWrite(S1_out2, LOW);
  digitalWrite(S1_out3, LOW);
  digitalWrite(S1_out4, HIGH);
   break;
   case 8: 
  digitalWrite(S1_out1, HIGH);// 8
  digitalWrite(S1_out2, LOW);
  digitalWrite(S1_out3, LOW);
  digitalWrite(S1_out4, HIGH);
   break;
  }
  stepper1_step_phase++;
  if (stepper1_step_phase == 9){
    stepper1_step_phase = 1;
    stepper1_count_steps--;
    //Serial.println("stepper1_count_steps : "+String (stepper1_count_steps));
    if (stepper1_count_steps == 0){
      Flag_stepper1_busy = false;
      Flag_timer_stepper1_phase = false;
      //set stepper at rest
      digitalWrite(S1_out1, LOW);
      digitalWrite(S1_out2, LOW);
      digitalWrite(S1_out3, LOW);
      digitalWrite(S1_out4, LOW);
//      Serial.println("stepper_1_done");
      //send response to app
      String stpR = ":S1/"+S1_cmdr_id+"/rest/;";
      send_respons_to_App(stpR);
      }else{
        //set timer again
        Timer_stepper1_phase = micros()+delay_stepper1_phase;
      }
  }else{
    //set timer again
    Timer_stepper1_phase = micros()+delay_stepper1_phase;
  } 
};

void do_stepper2_step(){//called when step_phase timer triger
int S2_i;
if (Flag_stepper2_clckw != true){
  S2_i = stepper2_step_phase;
  }else{
  S2_i = 8 - stepper2_step_phase+1;
  }
  
  switch (S2_i) {
   case 1:    
   digitalWrite(S2_out1, HIGH);// 1
   digitalWrite(S2_out2, LOW);
   digitalWrite(S2_out3, LOW);
   digitalWrite(S2_out4, LOW);
   break;
   case 2: 
  digitalWrite(S2_out1, HIGH);// 2 
  digitalWrite(S2_out2, HIGH);
  digitalWrite(S2_out3, LOW);
  digitalWrite(S2_out4, LOW);
  
  break;
   case 3: 
  digitalWrite(S2_out1, LOW);// 3 
  digitalWrite(S2_out2, HIGH);
  digitalWrite(S2_out3, LOW);
  digitalWrite(S2_out4, LOW);
   break;
   case 4: 
  digitalWrite(S2_out1, LOW);// 4 
  digitalWrite(S2_out2, HIGH);
  digitalWrite(S2_out3, HIGH);
  digitalWrite(S2_out4, LOW);
   break;
   case 5: 
  digitalWrite(S2_out1, LOW);// 5
  digitalWrite(S2_out2, LOW);
  digitalWrite(S2_out3, HIGH);
  digitalWrite(S2_out4, LOW);
   break;
   case 6: 
  digitalWrite(S2_out1, LOW);// 6
  digitalWrite(S2_out2, LOW);
  digitalWrite(S2_out3, HIGH);
  digitalWrite(S2_out4, HIGH);
   break;
   case 7: 
  digitalWrite(S2_out1, LOW);// 7
  digitalWrite(S2_out2, LOW);
  digitalWrite(S2_out3, LOW);
  digitalWrite(S2_out4, HIGH);
   break;
   case 8: 
  digitalWrite(S2_out1, HIGH);// 8
  digitalWrite(S2_out2, LOW);
  digitalWrite(S2_out3, LOW);
  digitalWrite(S2_out4, HIGH);
   break;
  }
  stepper2_step_phase++;
  if (stepper2_step_phase == 9){
    stepper2_step_phase = 1;
    stepper2_count_steps--;
    //Serial.println("S2 count_steps : "+String (stepper2_count_steps));
    
    if (stepper2_count_steps <= 0){
      //done all the steps (one step has 8 phases)
      Flag_stepper2_busy = false;
      Flag_timer_stepper2_phase = false;
      //set stepper at rest
      digitalWrite(S2_out1, LOW);
      digitalWrite(S2_out2, LOW);
      digitalWrite(S2_out3, LOW);
      digitalWrite(S2_out4, LOW);
      //Serial.println("stepper_2_done");
      //send response to app
      String stp2R = ":S2/"+S2_cmdr_id+"/rest/;";
      send_respons_to_App(stp2R);
      }else{
        //set timer again
        Timer_stepper2_phase = micros()+delay_stepper2_phase;
      }
  }else{
    //set timer again
    Timer_stepper2_phase = micros()+delay_stepper2_phase;
  } 
};

/*end of stepper routines */

void send_respons_to_App(String rs_data) {
  //send result to APP
  //Serial.println(rs_data);
  //String  report = "The Carousel is stationary ;";
  int rIdx = 0;
  int sLgth = rs_data.length();
  char nxtChr;
/*  while (rIdx != sLgth) {
    nxtChr = rs_data.charAt(rIdx);
    rIdx++;
    ESP_BT.write(nxtChr);*/
  };

void A_stop_the_carousel() {
  //drive the motor stop
  digitalWrite(A_out1, HIGH);
  digitalWrite(A_out2, LOW);
  //stop the timers that make the motor run
//x
//  A_Flag_delayH1_drive = false;
//  A_Flag_delayH2_drive = false;
//+
  FA_flagDriveDelay = false;
  
  //start timer wait for stop
  A_Timer_wait_for_stop = millis() + 200;
  //raise flag timer running
  A_Flag_wait_for_stop = true;
  A_counter_interupts = 0;
  FA_motor_mode = A_state_wait_stop;

};//end of --------------stop------------

void B_stop_the_carousel() {
  //drive the motor stop
  digitalWrite(B_out1, HIGH);
  digitalWrite(B_out2, LOW);
  //stop the timers that make the motor run
//x
//  B_Flag_delayH1_drive = false;
//  B_Flag_delayH2_drive = false;
  //start timer wait for stop
  B_Timer_wait_for_stop = millis() + 200;
  //raise flag timer running
  B_Flag_wait_for_stop = true;
  B_counter_interupts = 0;
  B_motor_mode = B_state_wait_stop;
};

//<<<<<<<< itrp >>>>>>>>>>>

//+
void FA_drive() {
//Serial.println("FA drive delay time up");
    //stop the timer  
    FA_flagDriveDelay = false;
    
   //test for clkw or aClkw
    if (FA_Flag_AClkw == true) {
      //now test for drive phase
      if (FA_drivePhase1 == true){
        //attach the interupt to RISING
        attachInterrupt(FA_hall1, FA_h1intrpRising, RISING);
        FA_drivePhase1 = false;
      }else{
        //attch interupt to FALLING
        attachInterrupt(FA_hall1, FA_h1intrpFalling, FALLING);
        FA_drivePhase1 = true;
      };
    
    }else{
    //now test for drive phase
      if (FA_drivePhase1 == true){
        //attach the interupt to RISING
        attachInterrupt(FA_hall1, FA_h1intrpRising, FALLING);
        FA_drivePhase1 = false;
      }else{
        //attch interupt to FALLING
        attachInterrupt(FA_hall1, FA_h1intrpFalling, RISING);
        FA_drivePhase1 = true;
      };
    };
};//end of FA_drive (old A_hl1Irp2)

//+
void FB_drive() {
//Serial.println("FB drive delay time up");
    //stop the timer  
    FB_flagDriveDelay = false;
    
   //test for clkw or aClkw
    if (FB_Flag_AClkw == true) {
      //now test for drive phase
      if (FB_drivePhase1 == true){
        //attach the interupt to RISING
        attachInterrupt(FB_hall1, FB_h1intrpRising, RISING);
        FB_drivePhase1 = false;
      }else{
        //attch interupt to FALLING
        attachInterrupt(FB_hall1, FB_h1intrpFalling, FALLING);
        FB_drivePhase1 = true;
      };
    
    }else{
    //now test for drive phase
      if (FB_drivePhase1 == true){
        //attach the interupt to RISING
        attachInterrupt(FB_hall1, FB_h1intrpRising, FALLING);
        FB_drivePhase1 = false;
      }else{
        //attch interupt to FALLING
        attachInterrupt(FB_hall1, FB_h1intrpFalling, RISING);
        FB_drivePhase1 = true;
      };
    };
};//end of FB_drive (old B_hl1Irp2)

void testIrp(){
//  Serial.println(">>>>>>> IRP FB h1 <<<<<< : " + String(digitalRead(FB_hall1)));//dBug
};

void loop() {//@@@@@@ General start of loop @@@@@@@@
if(flagSwitchWebsoc != true){
  //the flagSwitchWebsoc = true
  //WiFiClient client = server.available();
  hotSpotClient = hotSpot.available();
   if (hotSpotClient) {
      if(!flagCredentialDialogServed){
        //hotspotClient is available
        //Serial.println("just before serveCrdt page");
        serveCrdtPage();
      }
    }
}
  
//-----------------------
  //timestamp start of loop
  loop_time = millis();
  if (loop_time_store > 2){
//    Serial.println("loop_delay long "+String (loop_time_store));
  };
  
  if (Flag_test_timer == true && millis() >= Timer_test_time){
    Flag_test_timer = false;
    //Serial.println("loop_delay "+String (loop_time_store));
    Timer_test_time = millis()+2000;
    Flag_test_timer = true;

    if (FA_motor_mode == A_state_parked){
      if (FlagTestIrp == false){
        //attach test irp RISING to h1
        attachInterrupt(FB_hall1,testIrp,RISING);
        FlagTestIrp = true;
      }
    };

// Serial.println ("dBug FB h1 : " + String(digitalRead(FB_hall1)));//dBug
 
  };
   
  /* <<<<<<<<<<<<  loop motor driver - starts here >>>>>>>>>>*/
  if (A_Flag_active_cmnd == true) {
  
    if (FA_motor_mode == FA_sensor) {
      //Serial.println("A_motor_mode"+String(A_motor_mode);
      if (A_Flag_sensor != true) {      
        A_Flag_sensor = true;
      }
     }else if (FA_motor_mode == A_state_parked) {

    }else if (FA_motor_mode == A_state_running) {
      //when running check for drive delay timers
      if (FA_flagDriveDelay == true && millis() >= FA_timerDriveDelay) {
        //stop the timer
        
        //do drive motor    
        FA_drive();
      };

      FA_Flag_rev_calc = true;
  
      FA_Flag_started_error_stop_check = true;
      
     
    }else if (FA_motor_mode == A_state_standing) { //#### state
      //Serial.println("#### state standing ####");
      //reset all parameters
      A_motor_reset();
      //set motor mode to start
      FA_motor_mode = A_state_start;
      //do start procedure
      A_Flag_error_stop = false;
      FA_count_start = 4;
      A_do_start();//this is hitting the start button
    }else if (FA_motor_mode == A_state_start) { //#### state
      //Serial.println("#### state start ####");
      //check start sequence timers
      //check for start timer expiery
      if (FA_Flag_started != true) {
        if (FA_Flag_timer_spulse == true && millis() >= FA_Timer_Start_pulse) {
          A_do_start();
        }
      }
      //if started and stable set state to running
      if (FA_Flag_started == true) {
        FA_motor_mode = A_state_running;
      }
    }else if (FA_motor_mode == A_state_error_stop) {
      //reset and start again
      FA_motor_mode = A_state_standing;
    }else if (FA_motor_mode == A_state_error_drctn) {
      //reset and start again
      // it is not standing ??????
      FA_motor_mode = A_state_standing;
    
    }else if (FA_motor_mode == A_state_wait_stop) {

      if (A_Flag_wait_for_stop == true && millis() >= A_Timer_wait_for_stop) {
        //has motor stopped
        //Serial.println("wait_stop, timer expired");
        if (A_counter_interupts == 0) {
          //reset motor
          A_motor_reset();
          //re - set motor to apropriate state after stop
//          Serial.println("current command -- : " + String(current_command));
          if (current_command == shake_motor) {
            FA_motor_mode = A_state_shake;
          }else if (current_command == crawl_motor) {
            FA_motor_mode = A_state_crawl;
          }else if (current_command == stop_motor) {
            FA_motor_mode = A_state_parked;
//            Serial.println("set_motor_mode "+String(A_state_parked));
          }else if (current_command == pendulum_motor) {
            FA_motor_mode = A_state_pendulum;
          }else if (current_command == qrtr_step_motor) {
            FA_motor_mode = A_state_qrtr_step;
          }else if (current_command == swing_LR_motor) {
            FA_motor_mode = A_state_swing_LR;
          }else if (current_command == sensor_motor) {
            //A_motor_mode = FA_sensor;
            //Serial.println("A_motor_mode = A_state_swing_LR");
          }
          //disengage motor
          digitalWrite(A_out1, LOW);
          digitalWrite(A_out2, LOW);
          String Asr = ":FA/"+FA_cmdr_id+"/"+String(digitalRead(FA_hall1))+
          "/"+String(FA_motor_mode)+"/"+String(current_command)+"/;";
          send_respons_to_App(Asr);
        } else { //not yet standing
          //Serial.println("carousel has not stopped");
        }
      }
    }else if (FA_motor_mode == A_state_shake) {
      //Serial.println("state is shake, comand :" + String(current_command));
      //it arrived here after the motor stopped.
      if (A_Flag_shake_in_progres != true) {
        //then do shake
        A_do_shake();
        A_Flag_shake_in_progres = true;
      }
    }else if (FA_motor_mode == A_state_crawl) {
      //Serial.println("state is crawl, comand :" + String(current_command));
      //it arrived here after the motor stopped.
      if (A_Flag_crawl_in_progres != true) {
        //then do crawl
        A_do_crawl();
        A_Flag_crawl_in_progres = true;
      }
    }else if (FA_motor_mode == A_state_pendulum) {
      //Serial.println("state is pendulum, comand :"+String(current_command));
      //it arrived here after the motor stopped.
      if (A_Flag_pendulum_in_progres != true) {
        //then do pendulum
        A_do_pendulum();
        A_Flag_pendulum_in_progres = true;
      }
    }else if (FA_motor_mode == A_state_qrtr_step) {
      //Serial.println("state is qrtr_step");
      //it arrived here after the motor stopped.
      if (A_Flag_qrtr_step_in_progres != true) {
        //then do qrtr step
        FA_Qstep();
        A_Flag_qrtr_step_in_progres = true;
      }
    }else if (FA_motor_mode == A_state_swing_LR) {
      //Serial.println("A_motor_mode == A_state_swing_LR");
      if (A_Flag_swing_in_progres != true) {     
        A_Flag_swing_in_progres = true;
      }
    } else {
    //Serial.println("there is no active command");
   }

  //the rev counter
  A_Timer_rev_count();

  //error stop check
  if (FA_motor_mode == A_state_running) {
    A_do_error_stop_check();
    //A_Flag_started_error_stop_check = true;
  }
};

  if (B_Flag_active_cmnd == true) {
    
    if (B_motor_mode == FB_sensor) {
      //Serial.println("B_motor_mode"+String(B_motor_mode);
      if (B_Flag_sensor != true) {      
        B_Flag_sensor = true;
      }
    }else if (B_motor_mode == B_state_parked) {
      //Serial.println("state parked");
      //if (B_Flag_started == false) {
        //B_motor_reset();
        //Serial.println ("motor state PARKED");
//x
//+        
      }else if (B_motor_mode == B_state_running) {
      //when running check for drive delay timers
      if (FB_flagDriveDelay == true && millis() >= FB_timerDriveDelay) {
        //stop the timer
      
        //do drive motor    
        FB_drive();
      };

      
//      if (B_Flag_h2 == true && millis() >= B_timer_delay_h2) {
//        //attachInterrupt(B_FB_hall1,hl2Irp2,RISING);
//        B_hl2Irp2();//testing
//      }
      
      //start counting revs
      B_Flag_rev_calc = true;   
      B_Flag_started_error_stop_check = true;
      
    }else if (B_motor_mode == B_state_standing) { //#### state
      //Serial.println("#### state standing ####");
      //reset all parameters
      B_motor_reset();
      //set motor mode to start
      B_motor_mode = B_state_start;
      //do start procedure
      B_Flag_error_stop = false;
      B_count_start = 4;
      B_do_start();//this is hitting the start button
    }else if (B_motor_mode == B_state_start) { //#### state
      //Serial.println("#### state start ####");
      //check start sequence timers
      //check for start timer expiery
      if (B_Flag_started != true) {
        if (B_Flag_timer_spulse == true && millis() >= B_Timer_Start_pulse) {
          B_do_start();
        }
      }
      //if started and stable set state to running
      if (B_Flag_started == true) {
        B_motor_mode = B_state_running;
      }
    }else if (B_motor_mode == B_state_error_stop) {
      //reset and start again
      B_motor_mode = B_state_standing;
    }else if (B_motor_mode == B_state_error_drctn) {
      //reset and start again
      // it is not standing ??????
      B_motor_mode = B_state_standing;
    
    }else if (B_motor_mode == B_state_wait_stop) {

      if (B_Flag_wait_for_stop == true && millis() >= B_Timer_wait_for_stop) {
        //has motor stopped
        //Serial.println("state_stop, timer expired");
        if (B_counter_interupts == 0) {
          //reset motor
          B_motor_reset();
          //set motor in apropriate state
          //Serial.println("current command: " + String(current_command));
          if (current_command == shake_motor) {
            B_motor_mode = B_state_shake;
          }else if (current_command == crawl_motor) {
            B_motor_mode = B_state_crawl;
          }else if (current_command == stop_motor) {
            B_motor_mode = B_state_parked;
          }else if (current_command == pendulum_motor) {
            B_motor_mode = B_state_pendulum;
          }else if (current_command == qrtr_step_motor) {
            B_motor_mode = B_state_qrtr_step;
          }else if (current_command == swing_LR_motor) {
            B_motor_mode = B_state_swing_LR;
          }else if (current_command == sensor_motor) {
            B_motor_mode = FB_sensor;
          }
          //disengage motor
          digitalWrite(B_out1, LOW);
          digitalWrite(B_out2, LOW);
          String Bsr = ":FB/"+FB_cmdr_id+"/"+String(digitalRead(FB_hall1))+
          "/"+String(B_motor_mode)+"/"+String(current_command)+"/;";
          send_respons_to_App(Bsr);
        } else { //not yet standing
          //Serial.println("carousel has not stopped");
        }
      }
    }else if (B_motor_mode == B_state_shake) {
      //Serial.println("state is shake, comand :" + String(current_command));
      //it arrived here after the motor stopped.
      if (B_Flag_shake_in_progres != true) {
        //then do shake
        B_do_shake();
        B_Flag_shake_in_progres = true;
      }
    }else if (B_motor_mode == B_state_crawl) {
      //Serial.println("state is crawl, comand :" + String(current_command));
      //it arrived here after the motor stopped.
      if (B_Flag_crawl_in_progres != true) {
        //then do crawl
        B_do_crawl();
        B_Flag_crawl_in_progres = true;
      }
    }else if (B_motor_mode == B_state_pendulum) {
      //Serial.println("state is pendulum, comand :"+String(current_command));
      //it arrived here after the motor stopped.
      if (B_Flag_pendulum_in_progres != true) {
        //then do crawl
        B_do_pendulum();
        B_Flag_pendulum_in_progres = true;
      }
    }else if (B_motor_mode == B_state_qrtr_step) {
      //Serial.println("state is qrtr_step");
      //it arrived here after the motor stopped.
      if (B_Flag_qrtr_step_in_progres != true) {
        //then do qrtr step
        FB_Qstep();
        B_Flag_qrtr_step_in_progres = true;
      }
    }else if (B_motor_mode == B_state_swing_LR) {
      //Serial.println("B_motor_mode == B_state_swing_LR");
      if (B_Flag_swing_in_progres != true) {      
        B_Flag_swing_in_progres = true;
      }
    } else {
    //Serial.println("there is no B active command");
  }
};
  //the rev counter
  B_Timer_rev_count();

  //error stop check
  if (B_motor_mode == B_state_running) {
    B_do_error_stop_check();
    //B_Flag_started_error_stop_check = true;
  }

  /* ^^^^^^^^^^^^^^^^^   loop motor driver - ends here* ^^^^^^^^^^^^*/

  // ---------- check for complex command timer exp-------

     
//pemdulum    
  if (A_Flag_timer_pndlm == true && millis() >= A_time_pendlm){
    A_do_pendulum();
  };
  if (B_Flag_timer_pndlm == true && millis() >= B_time_pendlm){
    B_do_pendulum();
  };
  
  
  // if the shake timer flag is raised check if expired
  if (A_Flag_cmcShake == true && millis() >= A_Timer_cmcShake) {
    //Serial.println("shake time exp");
    //the timer is going off
    //if shake is true set timer again

    //drive the motor
    if (A_flipFlop == true) {
      //drive
      digitalWrite(A_out1, LOW);
      digitalWrite(A_out2, HIGH);
      A_flipFlop = false;
    } else {
      //drive
      digitalWrite(A_out1, HIGH);
      digitalWrite(A_out2, LOW);
      A_flipFlop = true;
    }
    //set the timer again for next shake
    A_Timer_cmcShake = millis() + Delay_cmcShake;
  }

  if (B_Flag_cmcShake == true && millis() >= B_Timer_cmcShake) {
    //Serial.println("shake time exp");
    //the timer is going off
    //if shake is true set timer again

    //drive the motor
    if (B_flipFlop == true) {
      //drive
      digitalWrite(B_out1, LOW);
      digitalWrite(B_out2, HIGH);
      B_flipFlop = false;
    } else {
      //drive
      digitalWrite(B_out1, HIGH);
      digitalWrite(B_out2, LOW);
      B_flipFlop = true;
    }
    //set the timer again for next shake
    B_Timer_cmcShake = millis() + Delay_cmcShake;
  }

  // if the crawl timer flag is raised check if expired
  if (A_Flag_cmc_Crawl == true && millis() >= A_Timer_cmcCrawl) {
    if (A_flipFlop == true) {
      //drive
      digitalWrite(A_out1, LOW);
      digitalWrite(A_out2, HIGH);
      A_flipFlop = false;
    } else {
      //drive
      digitalWrite(A_out1, HIGH);
      digitalWrite(A_out2, LOW);
      A_flipFlop = true;
    }
    A_Timer_cmcCrawl = millis() + Delay_cmcCrawl;
  }

  // if the crawl timer flag is raised check if expired
  if (B_Flag_cmc_Crawl == true && millis() >= B_Timer_cmcCrawl) {
    if (B_flipFlop == true) {
      //drive
      digitalWrite(B_out1, LOW);
      digitalWrite(B_out2, HIGH);
      B_flipFlop = false;
    } else {
      //drive
      digitalWrite(B_out1, HIGH);
      digitalWrite(B_out2, LOW);
      B_flipFlop = true;
    }
    B_Timer_cmcCrawl = millis() + Delay_cmcCrawl;
  }


//qS+
  if (FA_qStepGuard == true && millis() >= FA_qStepTimer) {
    Serial.println("qStep timer up"); 
    if (FA_qStepHL == true){
      FA_qStepHL = false;
      digitalWrite(A_out1, LOW);
      digitalWrite(A_out2, HIGH);
      FA_qStepTimer = millis() + 100;
    }else{
      if (FA_qStep_irp == true){
       //rest
       FA_qStepGuard = false;
       digitalWrite(A_out1, LOW);
       digitalWrite(A_out2, LOW);
       Serial.println("FA_qStep complete - rest"); 
      }else{
       FA_qStepGuard = false;
       Serial.println("FA_qStep fail no IRP"); 
      }
    }
  };

  // if B_Timer_qrtr_drive expiered - do again
  if (B_Flag_timer_qrtrd == true && millis() >= B_Timer_qrtr_drive) {
    if (B_flipFlop == true) {
      //drive
      digitalWrite(B_out1, HIGH);
      digitalWrite(B_out2, LOW);
      B_flipFlop = false;
    } else {
      //drive
      digitalWrite(B_out1, LOW);
      digitalWrite(B_out2, HIGH);
      B_flipFlop = true;
    }
    B_Timer_qrtr_drive = millis() + 100;
  }


  // if B_Timer_qrtrS_lock expiers
  if (B_Flag_delay_qrtrS_lock == true && millis() >= B_Timer_qrtrS_lock) {
    B_Flag_delay_qrtrS_lock = false;
    //put in rest
    digitalWrite(B_out1, LOW);
    digitalWrite(B_out2, LOW);
    //Serial.println("B motor in rest");
    B_motor_mode = B_state_parked;
    B_Flag_qrtr_step_in_progres = false;
    //respond_to_App = "BM/5/done;\n";#R#
    String Br = ":FB/"+FB_cmdr_id+"/"+String(digitalRead(FB_hall1))+"/"+String(B_motor_mode)+"/"+String(current_command)+"/;";
    send_respons_to_App(Br);
  }
  //timers  stepper1
  //if Timer_stepper_phase expires
  if (Flag_timer_stepper1_phase == true && micros() >= Timer_stepper1_phase){
    do_stepper1_step();
    //Serial.println("timer_fired_step_phase"+String(stepper_step_phase));
  }
  //test if stepper is free to do new command
  if (Flag_new_stepper1_command == true){
    //if (Flag_stepper1_busy != true){ //testing to see if stepper digrees will change after new command
      //Serial.println("executing new stepper command");
      //do the new command
      do_stepper1();
  }

  //timers  stepper2
  //if Timer_stepper_phase expires
  if (Flag_timer_stepper2_phase == true && micros() >= Timer_stepper2_phase){
    do_stepper2_step();
    //Serial.println("timer_fired_step_phase"+String(stepper_step_phase));
  }
  //test if stepper is free to do new command
  if (Flag_new_stepper2_command == true){
    //if (Flag_stepper2_busy != true){
      //Serial.println("executing new stepper command");
      //do the new command
      do_stepper2();
    //}
  }

  loop_time_store = millis() - loop_time;

  
//motor initialisation
if (Flag_motor_init == false){
//give turn commands
    cmnd_full = ":FA/none/1/100/;";
    interpret_command();
    delay(20);
    cmnd_full = ":FB/none/1/100/;";
    interpret_command();
    delay(20);   
    cmnd_full = ":FA/none/199/;";
    interpret_command();
    delay(20);
    cmnd_full = ":FB/none/199/;";
    interpret_command();
    delay(20);
      
//  Serial.println("FFB_hall1 "+String(digitalRead(FB_hall1))+"pin "+String(FB_hall1));
  
  Serial.println ("motor init complete");
  Serial.println ("FA state is "+String(FA_motor_mode));
  Flag_motor_init = true;
};

//lights module check for timers expiary
//test for raised flags - blink flags
if (FL_S1Ob){
  //test if delay before next blink is up
  if (millis() > TL_S1Ob){
        //toggle the output
    if (FL_S1O_on == true){
      digitalWrite(S1_out1,LOW);
      FL_S1O_on = false;
      }else{
      digitalWrite(S1_out1,HIGH);
       FL_S1O_on = true; 
      }
    //set timer pulse width
    TL_S1Op = millis()+Pw_S1O;
    //raise the puls timer flag
    FL_S1Op = true;
    //set the delay to the next blink
    TL_S1Ob = millis()+Bd_S1O;

  }else{
    //if pulse flag raised and time is up
    if(FL_S1Op==true){
      if (millis() >= TL_S1Op){     
      //out invert port
      if (FL_S1O_on == true){
      digitalWrite(S1_out1,LOW);
      FL_S1O_on = false;
      }else{
      digitalWrite(S1_out1,HIGH);
       FL_S1O_on = true; 
      }
      //the time is up drop flag
      FL_S1Op = false;
      }
    }
  }
 
}

if(FL_S1Yb){
  //test if delay before next blink is up
  if (millis() > TL_S1Yb){
        //toggle the output
    if (FL_S1Y_on == true){
      digitalWrite(S1_out2,LOW);
      FL_S1Y_on = false;
      }else{
      digitalWrite(S1_out2,HIGH);
       FL_S1Y_on = true; 
      }
    //set timer pulse width
    TL_S1Yp = millis()+Pw_S1Y;
    //raise the puls timer flag
    FL_S1Yp = true;
    //set the delay to the next blink
    TL_S1Yb = millis()+Bd_S1Y;

  }else{
    //if pulse flag raised and time is up
    if(FL_S1Yp==true){
      if (millis() >= TL_S1Yp){     
      //out invert port
      if (FL_S1Y_on == true){
      digitalWrite(S1_out2,LOW);
      FL_S1Y_on = false;
      }else{
      digitalWrite(S1_out2,HIGH);
       FL_S1Y_on = true; 
      }
      //the time is up drop flag
      FL_S1Yp = false;
      }
    }
  }
 
}

if(FL_S1Pb){
  //test if delay before next blink is up
  if (millis() > TL_S1Pb){
        //toggle the output
    if (FL_S1P_on == true){
      digitalWrite(S1_out3,LOW);
      FL_S1P_on = false;
      }else{
      digitalWrite(S1_out3,HIGH);
       FL_S1P_on = true; 
      }
    //set timer pulse width
    TL_S1Pp = millis()+Pw_S1P;
    //raise the puls timer flag
    FL_S1Pp = true;
    //set the delay to the next blink
    TL_S1Pb = millis()+Bd_S1P;

  }else{
    //if pulse flag raised and time is up
    if(FL_S1Pp==true){
      if (millis() >= TL_S1Pp){     
      //out invert port
      if (FL_S1P_on == true){
      digitalWrite(S1_out3,LOW);
      FL_S1P_on = false;
      }else{
      digitalWrite(S1_out3,HIGH);
       FL_S1P_on = true; 
      }
      //the time is up drop flag
      FL_S1Pp = false;
      }
    }
  }
   
} 

if(FL_S1Bb){
  //test if delay before next blink is up
  if (millis() > TL_S1Bb){
        //toggle the output
    if (FL_S1B_on == true){
      digitalWrite(S1_out4,LOW);
      FL_S1B_on = false;
      }else{
      digitalWrite(S1_out4,HIGH);
       FL_S1B_on = true; 
      }
    //set timer pulse width
    TL_S1Bp = millis()+Pw_S1B;
    //raise the puls timer flag
    FL_S1Bp = true;
    //set the delay to the next blink
    TL_S1Bb = millis()+Bd_S1B;

  }else{
    //if pulse flag raised and time is up
    if(FL_S1Bp==true){
      if (millis() >= TL_S1Bp){     
      //out invert port
      if (FL_S1B_on == true){
      digitalWrite(S1_out4,LOW);
      FL_S1B_on = false;
      }else{
      digitalWrite(S1_out4,HIGH);
       FL_S1B_on = true; 
      }
      //the time is up drop flag
      FL_S1Bp = false;
      }
    }
  }
  
}

if(FL_S2Ob){
   //test if delay before next blink is up
  if (millis() > TL_S2Ob){
        //toggle the output
    if (FL_S2O_on == true){
      digitalWrite(S2_out1,LOW);
      FL_S2O_on = false;
      }else{
      digitalWrite(S2_out1,HIGH);
       FL_S2O_on = true; 
      }
    //set timer pulse width
    TL_S2Op = millis()+Pw_S2O;
    //raise the puls timer flag
    FL_S2Op = true;
    //set the delay to the next blink
    TL_S2Ob = millis()+Bd_S2O;

  }else{
    //if pulse flag raised and time is up
    if(FL_S2Op==true){
      if (millis() >= TL_S2Op){     
      //out invert port
      if (FL_S2O_on == true){
      digitalWrite(S2_out1,LOW);
      FL_S2O_on = false;
      }else{
      digitalWrite(S2_out1,HIGH);
       FL_S2O_on = true; 
      }
      //the time is up drop flag
      FL_S2Op = false;
      }
    }
  }
  
}

if(FL_S2Yb){
  //test if delay before next blink is up
  if (millis() > TL_S2Yb){
        //toggle the output
    if (FL_S2Y_on == true){
      digitalWrite(S2_out2,LOW);
      FL_S2Y_on = false;
      }else{
      digitalWrite(S2_out2,HIGH);
       FL_S2Y_on = true; 
      }
    //set timer pulse width
    TL_S2Yp = millis()+Pw_S2Y;
    //raise the puls timer flag
    FL_S2Yp = true;
    //set the delay to the next blink
    TL_S2Yb = millis()+Bd_S2Y;

  }else{
    //if pulse flag raised and time is up
    if(FL_S2Yp==true){
      if (millis() >= TL_S2Yp){     
      //out invert port
      if (FL_S2Y_on == true){
      digitalWrite(S2_out2,LOW);
      FL_S2Y_on = false;
      }else{
      digitalWrite(S2_out2,HIGH);
       FL_S2Y_on = true; 
      }
      //the time is up drop flag
      FL_S2Yp = false;
      }
    }
  } 
}

if(FL_S2Pb){
  //test if delay before next blink is up
  if (millis() > TL_S2Pb){
        //toggle the output
    if (FL_S2P_on == true){
      digitalWrite(S2_out3,LOW);
      FL_S2P_on = false;
      }else{
      digitalWrite(S2_out3,HIGH);
       FL_S2P_on = true; 
      }
    //set timer pulse width
    TL_S2Pp = millis()+Pw_S2P;
    //raise the puls timer flag
    FL_S2Pp = true;
    //set the delay to the next blink
    TL_S2Pb = millis()+Bd_S2P;

  }else{
    //if pulse flag raised and time is up
    if(FL_S2Pp==true){
      if (millis() >= TL_S2Pp){     
      //out invert port
      if (FL_S2P_on == true){
      digitalWrite(S2_out3,LOW);
      FL_S2P_on = false;
      }else{
      digitalWrite(S2_out3,HIGH);
       FL_S2P_on = true; 
      }
      //the time is up drop flag
      FL_S2Pp = false;
      }
    }
  } 
}

if(FL_S2Bb){
  //test if delay before next blink is up
  if (millis() > TL_S2Bb){
        //toggle the output
    if (FL_S2B_on == true){
      digitalWrite(S2_out4,LOW);
      FL_S2B_on = false;
      }else{
      digitalWrite(S2_out4,HIGH);
       FL_S2B_on = true; 
      }
    //set timer pulse width
    TL_S2Bp = millis()+Pw_S2B;
    //raise the puls timer flag
    FL_S2Bp = true;
    //set the delay to the next blink
    TL_S2Bb = millis()+Bd_S2B;

  }else{
    //if pulse flag raised and time is up
    if(FL_S2Bp==true){
      if (millis() >= TL_S2Bp){     
      //out invert port
      if (FL_S2B_on == true){
      digitalWrite(S2_out4,LOW);
      FL_S2B_on = false;
      }else{
      digitalWrite(S2_out4,HIGH);
       FL_S2B_on = true; 
      }
      //the time is up drop flag
      FL_S2Bp = false;
      }
    }
  }
}

 //end of light module timers 
};//@@@@@@@@@@@   general end of loop @@@@@@@@@@


/*-------------new command interpreter --------------*/


void bldCmnd(char subCmnd) {
  //Serial.print("the subCmnd ");
  //Serial.println(subCmnd);
  
  if (subCmnd == ':') { // received a ":"/890/;";

    //it is the start of a new command
    //Serial.println("start of new command");
    //wait and receive all the characters
    //set flag buildingCommand
    flgBcmnd = true;
    //clear cmndString
    cmndSt = ":";

  } else { //sub command is not ':'
    if (subCmnd == ';') { // received ";"
      //it is the end of the command
      //Serial.println("it is the end of command");
      cmndSt = cmndSt + subCmnd;
      // drop flag build command in progress/890/;";

      flgBcmnd = false;
      Serial.println("Received full command > "+(cmndSt));
      //full command received now interpret
      cmnd_full = cmndSt;
      cmndSt ="";
      interpret_command();
      
      } else {
      //build the next character into the command (between : and ;)
      if (flgBcmnd == true) {
        cmndSt = cmndSt + subCmnd;
      }
    }
  }
}

void interpret_command(){

  if(cmnd_full.startsWith(":") && cmnd_full.endsWith(";")){
     //it is a valid command
    //Serial.println("cmnd start and end characters OK");

    //get the command segment 0/1/2/3/;
    int new_cmnd = get_segment(cmnd_full, '/',2).toInt();
    //Serial.println("new command "+String(new_cmnd));
    
    //get the qty of segments
    int qty_segments = get_qty_segments(cmnd_full,'/').toInt();
    //Serial.println("qty segments "+String(qty_segments));
    if (qty_segments >= 2){

    //get the device that will be commanded
    String device = get_segment(cmnd_full, '/', 0);
    //Serial.println("device "+device);

    //get the commander
    String cmnder = get_segment(cmnd_full, '/', 1);
    //Serial.println("cmnder "+cmnder);

    //test and assign device comander ID
    if (assign_cmnder(device,cmnder) == "ok"){
    
    if (new_cmnd <= 99){//motor spin
      //test if qty of segments is correct
      if (qty_segments == 4){
        //handle cmnd type A (3 segments)
        //Serial.println("cmnd type A 0 - 99");   
        xcut_cmnd_type_A(device,new_cmnd);
        }
    }else if (new_cmnd <= 199 && new_cmnd >= 100){//shake,stop,etc
        if (qty_segments == 3){
          //handle cmnd type B (2 segments)
          //Serial.println("cmnd type B 100 - 199");  
          xcut_cmnd_type_B(device,new_cmnd);
        }
    }else if (new_cmnd <= 299 && new_cmnd >= 200 ){//stepper
      if (qty_segments == 5){
        //handle cmnd type C (5 segments)
        //Serial.println("cmnd type C - 200 - 299");   
        xcut_cmnd_type_C(device,new_cmnd);
        }
    }else if (new_cmnd <= 399 && new_cmnd >= 300){//Info
      if (qty_segments == 3){
        //handle cmnd type D (3 segments)  
        //Serial.println("cmnd type D - 300 - 399");   
        xcut_cmnd_type_D(device,cmnder,new_cmnd); 
        }
    }else if (new_cmnd <= 499 && new_cmnd >= 400){//qrtsp,swing,etc
      if (qty_segments == 4){
        //handle cmnd type E (3 segments)
        //get 3de segment 0,1,2,3
        //get the device that will be commanded
        String sgmnt3 = get_segment(cmnd_full, '/', 3);
        //Serial.println("cmnd type E 400 - 499");   
        xcut_cmnd_type_E(device,new_cmnd,sgmnt3);
        }
     }else if (new_cmnd == 500){
      String s1 = get_segment(cmnd_full, ',', 1);
//      Serial.println("segment 2 "+s1);
      if(device == ":SB1R"){
        SB1R = ":"+s1;
        attachInterrupt(FA_hall1,A_sensor_h1_intrp_rise, RISING);
//        Serial.println("Sensor Buffer SB1R "+SB1R); 
      }else if(device == ":SB1F"){
        SB1F = ":"+s1;
        attachInterrupt(FA_hall1,A_sensor_h1_intrp_fall, FALLING);
//        Serial.println("Sensor Buffer SB1F "+SB1F); 
      }else if(device == ":SB2R"){
        SB2R = ":"+s1;
        attachInterrupt(FB_hall1,B_sensor_h1_intrp_rise, RISING);
//        Serial.println("Sensor Buffer SB1R "+SB2R); 
      }else if(device == ":SB2F"){
        SB2F = ":"+s1;
        attachInterrupt(FB_hall1,B_sensor_h1_intrp_fall, FALLING);
//        Serial.println("Sensor Buffer SB1F "+SB2F);
      }
     
//add lemo lights
      }else if (new_cmnd <= 699 && new_cmnd >= 600){
        //Serial.println("arrived at 2273 -- ");
        //get the light port no
        String segment3 = get_segment(cmnd_full, '/', 3);
        int port = segment3.toInt();
        
        //test if light on or off cmnd
        if (new_cmnd == 601 && device == ":S1"){
          switch (port) {
            case 1: digitalWrite(S1_out1, HIGH);
            FL_S1Ob = false;
            break;
            case 2: digitalWrite(S1_out2, HIGH);
            FL_S1Yb = false;
            break;
            case 3: digitalWrite(S1_out3, HIGH);
            FL_S1Pb = false;
            break;
            case 4: digitalWrite(S1_out4, HIGH);
            FL_S1Bb = false; 
            break;
          } 
        }else if (new_cmnd == 600 && device == ":S1"){                
          switch (port) {
            case 1: digitalWrite(S1_out1, LOW);
              FL_S1Ob = false;
              break;
            case 2: digitalWrite(S1_out2, LOW);
              FL_S1Yb = false; 
            break;
            case 3: digitalWrite(S1_out3, LOW);
              FL_S1Pb = false; 
            break;
            case 4: digitalWrite(S1_out4, LOW);
              FL_S1Bb = false; 
            break;
          } 
        }else if (new_cmnd == 601 && device == ":S2"){
          switch (port) {
            case 1: digitalWrite(S2_out1, HIGH);
             FL_S2Ob = false;
            break;
            case 2: digitalWrite(S2_out2, HIGH);
             FL_S2Yb = false;
            break;
            case 3: digitalWrite(S2_out3, HIGH);
             FL_S2Pb = false;
            break;
            case 4: digitalWrite(S2_out4, HIGH);
             FL_S2Bb = false;
            break;
          }
        }else if (new_cmnd == 600 && device == ":S2"){
          switch (port) {
            case 1: digitalWrite(S2_out1, LOW);
              FL_S2Ob = false; 
            break;
            case 2: digitalWrite(S2_out2, LOW);
              FL_S2Yb = false;
            break;
            case 3: digitalWrite(S2_out3, LOW);
              FL_S2Pb = false;
            break;
            case 4: digitalWrite(S2_out4, LOW);
              FL_S2Bb = false;
            break;
          } 
        }
  
        //test if blinking command
        if(new_cmnd == 603 || new_cmnd == 604){       
        //get the blink for duration in steps of 10ms
        String segment4 = get_segment(cmnd_full, '/', 4);
        //get the pulse duration in steps of 1ms
        String segment5 = get_segment(cmnd_full, '/', 5);
       
        int Bd = segment4.toInt();
        int Pd = segment5.toInt();
        

      if(device == ":S1"){                   
         switch (port) {
   case 1: 
    //save the durations for S1O 
    Bd_S1O = Bd*10;
    Pw_S1O = Pd;
    //raise the blinking duration timer flag
    FL_S1Ob=true;
    //load the timer
    TL_S1Ob=millis()+Bd_S1O;
    //drop the pulse flag
    FL_S1Op=false;
    if (new_cmnd == 603){
      //out HIGH
      digitalWrite(S1_out1,LOW);
      FL_S1O_on=false;
      }else{
        digitalWrite(S1_out1,HIGH);
        FL_S1O_on=true;
        };
   break;   
   case 2: 
    //save the durations for S1Y 
    Bd_S1Y = Bd*10;
    Pw_S1Y = Pd;
    //raise the blinking duration timer flag
    FL_S1Yb=true;
    //load the timer
    TL_S1Yb=millis()+Bd_S1Y;
    //drop the pulse flag
    FL_S1Yp=false;
    if (new_cmnd == 603){
      //out HIGH
      digitalWrite(S1_out2,LOW);
      FL_S1Y_on=false;
      }else{
        digitalWrite(S1_out2,HIGH);
        FL_S1Y_on=true;
        };
   break;
   case 3:
    //save the durations for S1O 
    Bd_S1P = Bd*10;
    Pw_S1P = Pd;
    //raise the blinking duration timer flag
    FL_S1Pb=true;
    //load the timer
    TL_S1Pb=millis()+Bd_S1P;
    //drop the pulse flag
    FL_S1Pp=false;
    if (new_cmnd == 603){
      //out HIGH
      digitalWrite(S1_out3,LOW);
      FL_S1P_on=false;
      }else{
        digitalWrite(S1_out3,HIGH);
        FL_S1P_on=true;
        };
  
   break;
   case 4: 
    //save the durations for S1O 
    Bd_S1B = Bd*10;
    Pw_S1B = Pd;
    //raise the blinking duration timer flag
    FL_S1Bb=true;
    //load the timer
    TL_S1Bb=millis()+Bd_S1B;
    //drop the pulse flag
    FL_S1Bp=false;
    if (new_cmnd == 603){
      //out HIGH
      digitalWrite(S1_out4,LOW);
      FL_S1B_on=false;
      }else{
        digitalWrite(S1_out4,HIGH);
        FL_S1B_on=true;
        }; 
   break;
         }
           
      }else if(device == ":S2"){

         switch (port) {
   case 1: 
    //save the durations for S1O 
    Bd_S2O = Bd*10;
    Pw_S2O = Pd;
    //raise the blinking duration timer flag
    FL_S2Ob=true;
    //load the timer
    TL_S2Ob=millis()+Bd_S2O;
    //drop the pulse flag
    FL_S2Op=false;
    if (new_cmnd == 603){
      //out HIGH
      digitalWrite(S2_out1,LOW);
      FL_S2O_on=false;
      }else{
        digitalWrite(S2_out1,HIGH);
        FL_S2O_on=true;
        };
   break;   
   case 2: 
    //save the durations for S1Y 
    Bd_S2Y = Bd*10;
    Pw_S2Y = Pd;
    //raise the blinking duration timer flag
    FL_S2Yb=true;
    //load the timer
    TL_S2Yb=millis()+Bd_S1Y;
    //drop the pulse flag
    FL_S2Yp=false;
    if (new_cmnd == 603){
      //out HIGH
      digitalWrite(S2_out2,LOW);
      FL_S2Y_on=false;
      }else{
        digitalWrite(S2_out2,HIGH);
        FL_S2Y_on=true;
        };
   break;
   case 3:
    //save the durations for S1O 
    Bd_S2P = Bd*10;
    Pw_S2P = Pd;
    //raise the blinking duration timer flag
    FL_S2Pb=true;
    //load the timer
    TL_S2Pb=millis()+Bd_S2P;
    //drop the pulse flag
    FL_S2Pp=false;
    if (new_cmnd == 603){
      //out HIGH
      digitalWrite(S2_out3,LOW);
      FL_S2P_on=false;
      }else{
        digitalWrite(S2_out3,HIGH);
        FL_S2P_on=true;
        };
  
   break;
   case 4: 
    //save the durations for S1O 
    Bd_S2B = Bd*10;
    Pw_S2B = Pd;
    //raise the blinking duration timer flag
    FL_S2Bb=true;
    //load the timer
    TL_S2Bb=millis()+Bd_S2B;
    //drop the pulse flag
    FL_S2Bp=false;
    if (new_cmnd == 603){
      //out HIGH
      digitalWrite(S2_out4,LOW);
      FL_S2B_on=false;
      }else{
        digitalWrite(S2_out4,HIGH);
        FL_S2B_on=true;
        }; 
   break;
         }      
      
      }      
      }        
      }    
        
    }else{
//     Serial.println("error assign cmndr "+cmnder);
    }
   }else{
    //not correct no of segments
//    Serial.println("error no of segments");
   }
  }else{
    //command not :----;
//    Serial.println("error start and stop char");
  }
};


String get_segment(String data, char separator, int index){
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i_sgmnt=0; i_sgmnt<=maxIndex && found<=index; i_sgmnt++){
    if(data.charAt(i_sgmnt) == separator || i_sgmnt == maxIndex){
      
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i_sgmnt == maxIndex) ? i_sgmnt+1 : i_sgmnt;
      //Serial.println("cmnd_items "+String(data.substring(strIndex[0], strIndex[1])));  
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
};

String get_qty_segments(String data, char separator){
  int sgmnt_qty = 0;
  int maxIndex = data.length()-1;
  for(int i_qty=0; i_qty <= maxIndex ; i_qty++){
    if(data.charAt(i_qty) == separator){
        sgmnt_qty++; 
    }
  }
     //Serial.println("how many segments found : "+String(sgmnt_qty));
     return String(sgmnt_qty);
};


String assign_cmnder(String cmnd_device,String new_cmnder){
    //get the requesting cmndr ID
       String prvs_id = "----";
       
       String result = "fail";
       if (cmnd_device == ":FA" && (FA_cmdr_id == "none" || FA_cmdr_id == new_cmnder)){
        prvs_id = FA_cmdr_id;
        FA_cmdr_id = new_cmnder;
        result = "ok";
      }else if (cmnd_device == ":FB" && (FB_cmdr_id == "none" || FB_cmdr_id == new_cmnder)){
        prvs_id = FB_cmdr_id;
        FB_cmdr_id = new_cmnder;
        result = "ok";
      }else if (cmnd_device == ":S1" && (S1_cmdr_id == "none" || S1_cmdr_id == new_cmnder)){
        prvs_id = S1_cmdr_id;
        S1_cmdr_id = new_cmnder;
        result = "ok";
      }else if (cmnd_device == ":S2" && (S2_cmdr_id == "none" || S2_cmdr_id == new_cmnder)){
        prvs_id = S2_cmdr_id;
        S2_cmdr_id = new_cmnder;
         result = "ok";
      }else if (cmnd_device == ":SB1R" && (SB1R_cmdr_id == "none" || SB1R_cmdr_id == new_cmnder)){
        prvs_id = SB1R_cmdr_id;
        SB1R_cmdr_id = new_cmnder;
//        Serial.println("assign new cmnder to SB1R is "+SB1R_cmdr_id);
        result = "ok";
      }else if (cmnd_device == ":SB1F" && (SB1F_cmdr_id == "none" || SB1F_cmdr_id == new_cmnder)){
        prvs_id = SB1F_cmdr_id;
        SB1F_cmdr_id = new_cmnder;
//        Serial.println("assign new cmnder to SB1F is "+SB1F_cmdr_id);
        result = "ok";
      }else if (cmnd_device == ":SB2R" && (SB2R_cmdr_id == "none" || SB2R_cmdr_id == new_cmnder)){
        prvs_id = SB2R_cmdr_id;
        SB2R_cmdr_id = new_cmnder;
//        Serial.println("assign new cmnder to SB2R is "+SB2R_cmdr_id);
        result = "ok";
      }else if (cmnd_device == ":SB2F" && (SB2F_cmdr_id == "none" || SB2F_cmdr_id == new_cmnder)){
        prvs_id = SB2F_cmdr_id;
        SB2F_cmdr_id = new_cmnder;
//        Serial.println("assign new cmnder to SB2F is "+SB2F_cmdr_id);
        result = "ok";
        
  }
  if (result == "fail"){
//    Serial.println("the device is"+cmnd_device);
//    Serial.println("cmnder of FA is "+FA_cmdr_id);
//    Serial.println("cmnder of FB is "+FB_cmdr_id);
//    Serial.println("cmnder of S1 is "+S1_cmdr_id);
//    Serial.println("cmnder of S2 is "+S2_cmdr_id);
//    Serial.println("cmnder of SB1R is "+SB1R_cmdr_id);
//    Serial.println("cmnder of SB1F is "+SB1F_cmdr_id);
//    Serial.println("cmnder of SB2R is "+SB1R_cmdr_id);
//    Serial.println("cmnder of SB2F is "+SB1F_cmdr_id);
//    Serial.println("requesting cmnder id is "+new_cmnder);   
  }
  //Serial.println(result+" commander "+cmnd_device+" reqsting "+new_cmnder +" previous "+prvs_id+";");//
  return result;
};

void xcut_cmnd_type_A(String ctA_device,int cta_new_cmnd){
    //get the required spead
    int spd = get_segment(cmnd_full, '/', 3).toInt();
    //this determines the speed
    int interval = map(spd, 0, 999, 60, 1); //make big number fast and small slow
    //Serial.println("maped motor pulse interval "+String(interval));
    if (ctA_device == ":FA" && (cta_new_cmnd == 0 || cta_new_cmnd == 1)){
      A_interval_drive = interval;
      //Serial.println("XXXX 123 XXX");
      //--------launch --------
       current_command = run_motor;
       A_Flag_active_cmnd = true;
       FA_motor_mode = A_state_standing;
      /*now its time to cancel all other comand related timers in
        preparation to implement new command*/
       A_Flag_cmcShake = false;
       A_Flag_cmc_Crawl = false;
//sx       A_Flag_A_swing_timer = false;
       A_Flag_qrtr_step = false;
       //stop the pendulim interupts
       detachInterrupt(FA_hall1);
//x       detachInterrupt(A_hall2);
       if (cta_new_cmnd == 1){
        FA_Flag_AClkw = false;
       }else{
        FA_Flag_AClkw = true;
       }
     }else if (ctA_device == ":FB" && (cta_new_cmnd == 0 || cta_new_cmnd == 1)){
        B_interval_drive = interval;
        //Serial.println(":FB values are set "+String(B_Flag_AClkw)+" "+String(B_interval_drive));
        //--------launch --------
       current_command = run_motor;
       B_Flag_active_cmnd = true;
       B_motor_mode = A_state_standing;
      /*now its time to cancel all other comand related timers in
        preparation to implement new command*/
       B_Flag_cmcShake = false;
       B_Flag_cmc_Crawl = false;
//sx       B_Flag_B_swing_timer = false;
       B_Flag_qrtr_step = false;
       //stop the pendulim interupts
       detachInterrupt(FB_hall1);
//x       detachInterrupt(B_hall2);
       if (cta_new_cmnd == 1){
        FB_Flag_AClkw = false;
       }else{
        FB_Flag_AClkw = true;
       }
    }else{
//      Serial.println("ERROR cmnd or device wrong");
      }
};//end xcut_cmnd_type_A

 
void xcut_cmnd_type_B(String ctB_device,int ctB_new_cmnd) {//mod cmnd
    
  /*is it a complex motor command of type B 
   * example :FA/XX/102/;
  102 shake
  103 crawl
  104 dance
  105 pendulum - change to 424
  106 quarter step
  107 
  108 not used
  170 swing L
  171 swing R
  180 sensorR
  181 sensorA
  182 sensor no spring triger on rise and fall
  199 stop*/

  //test what command it is
  
    if (ctB_new_cmnd == 102) {
      //Serial.println("it is cmnd 102 for "+ctB_device);    
      if(ctB_device == ":FA"){
        current_command = shake_motor;
        A_Flag_active_cmnd = true;
        A_stop_the_carousel();
        //Serial.println(ctB_device+""+current_command);
      }
      if(ctB_device == ":FB"){
        current_command = shake_motor;
        B_Flag_active_cmnd = true;
        B_stop_the_carousel();
        //Serial.println(ctB_device+""+current_command);
      }
         
    }else if (ctB_new_cmnd == 103) {      
      if(ctB_device == ":FA"){
       current_command = crawl_motor;
       A_Flag_active_cmnd = true;
       A_stop_the_carousel(); 
      }
      if(ctB_device == ":FB"){
       current_command = crawl_motor;
       B_Flag_active_cmnd = true;
       B_stop_the_carousel(); 
      }
      
    }else if (ctB_new_cmnd == 104) {     
      if(ctB_device == ":FA"){
       current_command = dance_motor;
       A_Flag_active_cmnd = true;
       A_stop_the_carousel(); 
      }
      if(ctB_device == ":FB"){
       current_command = dance_motor;
       B_Flag_active_cmnd = true;
       B_stop_the_carousel(); 
      }
        
    }else if (ctB_new_cmnd == 106) {
      if(ctB_device == ":FA"){
        current_command = qrtr_step_motor;
        A_Flag_active_cmnd = true;
        A_stop_the_carousel();
      }
      if(ctB_device == ":FB"){
        current_command = qrtr_step_motor;
        B_Flag_active_cmnd = true;
        B_stop_the_carousel();
      } 
        
    }else if (ctB_new_cmnd == 170 || ctB_new_cmnd == 171) {
      if (ctB_new_cmnd == 170 && ctB_device == ":FA"){
        A_swing_Aclkw();
       }else if (ctB_new_cmnd == 171 && ctB_device == ":FA"){
        A_swing_Clkw();
        }else if (ctB_new_cmnd == 170 && ctB_device == ":FB"){
        B_swing_Aclkw();
        }else if (ctB_new_cmnd == 171 && ctB_device == ":FB"){
        B_swing_Aclkw();
      };
     
    }else if (ctB_new_cmnd == 199 && ctB_device == ":FA") {//stop commannd for FA
      //Serial.println("arrived at M9 stop command");
      current_command = stop_motor;
      //Serial.println("stop requested for "+String(ctB_device));
      A_stop_the_carousel(); 
      
    }else if (ctB_new_cmnd == 199 && ctB_device == ":FB") {//stop commannd for FA
      //Serial.println("arrived at M9 stop command");
      current_command = stop_motor;
      //Serial.println("stop requested for "+String(ctB_device));
      B_stop_the_carousel();
      
    }else if (ctB_new_cmnd == 199 && ctB_device == ":S1") {//stop commannd for S1
      //Serial.println("stop requested for "+String(ctB_device));
      String S1rs;
        if (Flag_stepper1_busy == false){
        //respond to app already at rest
          S1rs = ":S1/"+S1_cmdr_id+"/rest/already_rest/;";
        }else{
          S1rs = ":S1/"+S1_cmdr_id+"/stop_rq/steps_comp/"+String(stepper1_count_steps)+"/;";
          //stop stepper1
          stepper1_count_steps = 1; //one last step will be done
          Flag_stepper1_busy = false;
          Flag_new_stepper1_command = false;
        }
        send_respons_to_App(S1rs);
        
      }else if (ctB_new_cmnd == 199 && ctB_device == ":S2") {//stop commannd for S2
      //Serial.println("stop requested for "+String(ctB_device));
      String S2rs;
        if (Flag_stepper2_busy == false){
        //respond to app already at rest
          S2rs = ":S2/"+S2_cmdr_id+"/rest/already_rest/;";
        }else{
          S2rs = ":S2/"+S2_cmdr_id+"/stop_rq/steps_comp/"+String(stepper2_count_steps)+"/;";
          //stop stepper1
          stepper2_count_steps = 1; //one last step will be done
          Flag_stepper2_busy = false;
          Flag_new_stepper2_command = false;
         }
         send_respons_to_App(S2rs);
         
      }
      
    if (ctB_new_cmnd == 181 || ctB_new_cmnd == 180 || ctB_new_cmnd == 182) { //182 no spring
        
        //if in parked continue
        if (ctB_device == ":FA"){// && A_motor_mode == A_state_parked){// || A_motor_mode == FA_sensor
         if (FA_motor_mode == A_state_parked){
          current_command = sensor_motor;
          FA_motor_mode = FA_sensor;
          A_Flag_active_cmnd = true;
          //setup sensor mode send respons when hall 1 change, setup interupt.
          //Serial.println("sensor to be set");
          //attachInterrupt(A_hall1,A_sensor_h1_intrp, RISING); 
          if (ctB_new_cmnd == 180){
            //engage the motor at rest hall1 is 0
            digitalWrite(A_out1, LOW);
            digitalWrite(A_out2, HIGH);
            //Serial.println("FA assigned as sensor A");
          }else if (ctB_new_cmnd == 181){
            //engage the motor at rest hall1 is 1
            digitalWrite(A_out1, HIGH);
            digitalWrite(A_out2, LOW);
            //Serial.println("FA assigned as sensor R");
          }else if (ctB_new_cmnd == 182){
          
          }else{
            A_Flag_active_cmnd = false;
//            Serial.println("not possible to assign sensor to "+ctB_device);
            }
           }
         } 
       } 
                
   if (ctB_device == ":FB"){
    //if in parked continue
        if (ctB_device == ":FB"){
         if (B_motor_mode == B_state_parked){
          current_command = sensor_motor;
          B_motor_mode = FB_sensor;
          B_Flag_active_cmnd = true;
          //setup sensor mode send respons when hall 1 change, setup interupt.
          //Serial.println("FB to be set as sensor");
          //attachInterrupt(FB_hall1,B_sensor_h1_intrp, RISING); 
          if (ctB_new_cmnd == 180){
            //engage the motor at rest hall1 is 0
            digitalWrite(B_out1, LOW);
            digitalWrite(B_out2, HIGH);
            //Serial.println("FB assigned as sensor A");
          }else if (ctB_new_cmnd == 181){
            //engage the motor at rest hall1 is 1
            digitalWrite(B_out1, HIGH);
            digitalWrite(B_out2, LOW);
            //Serial.println("FB assigned as sensor R");
          }else if (ctB_new_cmnd == 182){
            
        }else{
          B_Flag_active_cmnd = false;
//          Serial.println("not possible to assign sensor to "+ctB_device);
           }
       }
     }
   }       
  //Serial.println("xxx");
};
 
void xcut_cmnd_type_C(String ctC_device,int ctC_new_cmnd){
    //Serial.println("arrived at xcut_cmnd_type_C");
    //get segment 3 (0/1/2/3/)
    int Cspd = get_segment(cmnd_full, '/',3).toInt();
    //this determines the speed
    int ctC_delay = map(Cspd, 1, 999, 10000, 700); //make big number fast and small slow
    //Serial.println("spead "+String(Cspd)+" translated to delay "+String (ctC_delay));
    //get the command segment 0/1/2/3/;
    int Cdgrs = get_segment(cmnd_full, '/',4).toInt();
    //Serial.println("digrees "+String(Cdgrs));
    
    if (ctC_new_cmnd == 200 && ctC_device == ":S1"){
      //Serial.println("it is step cmnd for "+String(ctC_device));
      
      //SSS fix to stop rest respons to app
      
      step1_digrees = Cdgrs;
      stepper1_step_phase = 1;
      stepper1_count_steps = step1_digrees;
      //this determines the speed     
      delay_stepper1_phase = ctC_delay;
      
      Flag_stepper1_clckw = false;
      Flag_new_stepper1_command = true;
      Flag_stepper1_busy = false; //temp solution overide the current steps
      
     }else if (ctC_new_cmnd == 201 && ctC_device == ":S1"){
      //Serial.println("it is step cmnd for "+String(ctC_device));

      //SSS fix to stop rest respons to app
      
      step1_digrees = Cdgrs;
      stepper1_step_phase = 1;
      stepper1_count_steps = step1_digrees;
      //this determines the speed
      delay_stepper1_phase = ctC_delay;
      Flag_stepper1_clckw = true;
      Flag_new_stepper1_command = true;
      Flag_stepper2_busy = false; //temp solution overide the current steps
      //Serial.println(millis()-measure_delay);
      
     }else if (ctC_new_cmnd == 200 && ctC_device == ":S2"){
      //Serial.println("it is step cmnd for "+String(ctC_device));

      step2_digrees = Cdgrs;
      stepper2_step_phase = 1;
      stepper2_count_steps = step2_digrees;     
      //this determines the speed
      delay_stepper2_phase = ctC_delay;
      Flag_stepper2_clckw = false;
      Flag_new_stepper2_command = true;
      
     }else if (ctC_new_cmnd == 201 && ctC_device == ":S2"){
      //Serial.println("it is step cmnd for "+String(ctC_device));

      step2_digrees = Cdgrs;
      stepper2_step_phase = 1;
      stepper2_count_steps = step2_digrees;     
      //this determines the speed
      delay_stepper2_phase = ctC_delay;
      Flag_stepper2_clckw = true;
      Flag_new_stepper2_command = true;     
      }
 };//end xcut_cmnd_type_C


void xcut_cmnd_type_D(String ctD_device,String cmndr,int ctD_new_cmnd){
    String D_respons = "";
    //Serial.println("arrived at xcut_cmnd_type_D");
    if (ctD_new_cmnd == 399 && ctD_device == ":FA"){
      //release ownership
      FA_cmdr_id = "none";
      //Serial.println("commander set to none for"+String(ctD_device));
     }else if (ctD_new_cmnd == 399 && ctD_device == ":FB"){
      FB_cmdr_id = "none";
      //Serial.println("commander set to none for"+String(ctD_device));
     }else if (ctD_new_cmnd == 399 && ctD_device == ":S1"){
      S1_cmdr_id = "none";
      //Serial.println("commander set to none for"+String(ctD_device));
     }else if (ctD_new_cmnd == 399 && ctD_device == ":S2"){
      S2_cmdr_id = "none";
     }else if (ctD_new_cmnd == 399 && ctD_device == ":SB1R"){
      SB1R_cmdr_id = "none";
     }else if (ctD_new_cmnd == 399 && ctD_device == ":SB1F"){
      SB1F_cmdr_id = "none";
     }else if (ctD_new_cmnd == 399 && ctD_device == ":SB2R"){
      SB2R_cmdr_id = "none";
     }else if (ctD_new_cmnd == 399 && ctD_device == ":SB2F"){
      SB2F_cmdr_id = "none";
      
     }else if (ctD_new_cmnd == 300 && ctD_device == ":FA"){ //request for combo info
        int Asp = map(A_interval_drive,60,1,0, 999); //translate back
        D_respons = ":FA/"+cmndr+"/300/"+String(FA_motor_mode)+"/"+String(FA_Flag_AClkw)+"/"+String(Asp)+
        "/"+String(digitalRead(FA_hall1))+"/"+"/;";
//        Serial.println("combo FA >"+D_respons);    
     }else if (ctD_new_cmnd == 300 && ctD_device == ":FB"){
        int Bsp = map(B_interval_drive,60,1,0,999); //translate back
       D_respons = ":FB/"+cmndr+"/300/"+String(B_motor_mode)+"/"+String(FB_Flag_AClkw)+"/"+String(Bsp)+
        "/"+String(digitalRead(FB_hall1))+"/"+"/;";
//        Serial.println("combo FB >"+D_respons);
     }else if (ctD_new_cmnd == 300 && ctD_device == ":S1"){
      int S1sp = map(delay_stepper1_phase, 10000, 700, 1, 999); // 1, 999, 10000, 700
       D_respons = ":S1/"+cmndr+"/300/"+String(Flag_stepper1_busy)+"/"+String(S1sp)+"/"+String(step1_digrees)+"/"+Flag_stepper1_clckw+
        "/"+String(stepper1_count_steps)+"/;";
        //Serial.println("combo S1 >"+respond_to_App);
     }else if (ctD_new_cmnd == 300 && ctD_device == ":S2"){
      int S2sp = map(delay_stepper2_phase, 10000, 700, 1, 999); // 1, 999, 10000, 700
       D_respons = ":S2/"+cmndr+"/300/"+String(Flag_stepper2_busy)+"/"+String(S2sp)+"/"+String(step2_digrees)+"/"+Flag_stepper2_clckw+
        "/"+String(stepper2_count_steps)+"/;";
        //Serial.println("combo S2 >"+respond_to_App);    
        
     }else if (ctD_new_cmnd == 301 && ctD_device == ":FA"){ //request for combo info
       //request for motor mode
          D_respons = ":FA/motor_mode/"+String(FA_motor_mode)+";";      
     }else if (ctD_new_cmnd == 301 && ctD_device == ":FB"){
       //request for motor mode
          D_respons = ":FB/motor_mode/"+String(B_motor_mode)+";";
     }
    send_respons_to_App(D_respons);
    Serial.println("respons >>"+D_respons);
 };//end xcut_cmnd_type_D

 void xcut_cmnd_type_E(String ctE_device,int ctE_new_cmnd,String sgm4){
    //Serial.println("arrived at xcut_cmnd_type_D");
    if (ctE_new_cmnd == 400 && ctE_device == ":FA"){
      //Serial.println("info combo for "+String(ctE_device));
      
     }else if (ctE_new_cmnd == 411 && ctE_device == ":FA"){
        current_command = qrtr_step_motor;
        FA_qStClckw = sgm4;
        A_Flag_active_cmnd = true;
        A_stop_the_carousel();
        A_Flag_qrtr_step_in_progres = false;
      Serial.println("qrtr step"+sgm4+" for "+ctE_device);

     }else if (ctE_new_cmnd == 411 && ctE_device == ":FB"){
        current_command = qrtr_step_motor;
        B_Flag_active_cmnd = true;
        B_stop_the_carousel();
        B_Flag_qrtr_step_in_progres = false;
      //Serial.println("qrtr step"+sgm4+" for "+ctE_device);
      
     }else if (ctE_new_cmnd == 424 && ctE_device == ":FA") {
        current_command = pendulum_motor;
        A_Flag_active_cmnd = true;
        A_stop_the_carousel();
        A_pendulum_time = sgm4.toInt();
     }else if(ctE_new_cmnd == 424 && ctE_device == ":FB"){
        current_command = pendulum_motor;
        B_Flag_active_cmnd = true;
        B_stop_the_carousel();
        B_pendulum_time = sgm4.toInt();
       
    }
 };//end xcut_cmnd_type_E

//----------------------------
bool getCredentials(){
   bool valid = false;
   String ID;
   String PW;
   //read the wifiCrdt.txt
   File crd = SPIFFS.open("/wifiCrdt.txt");
   if(crd){
     //the file is open
     Serial.println(crd.size());
     String content;
     while(crd.available()){
      char c = crd.read();
      //Serial.write(c);
      content = String(content + c);
        }
      crd.close();
        
     //test content for credentials
     if(!content.indexOf("SSID")){ 
       //the file contains credentials
       userCredentials = content;
       //split content into SSID and password
       //SSID=Ggggg&PASSWORD=Bbbbbb HTTP/1.1
       //find the index of SSID
       int sI = userCredentials.indexOf("SSID=")+5;
       int pI = userCredentials.indexOf("&PASSWORD=");
       int sp = userCredentials.indexOf(" ");      
       //String uId = userCredentials.substring(sI, pI);
       userSsid = userCredentials.substring(sI, pI);
       //const char *c = str.c_str();
       //userSsid = uId.c_str();
       //userPassword = uPw.c_str();
       userPassword = userCredentials.substring(pI+10, sp);
       //Serial.println("!!!! ssid > "+userSsid+"  password > "+userPassword);
       valid = true;  
            
     }else{
       //the file does not contain credentials
       //Serial.println("the file does not contain credentials");
       userCredentials = "the file does not contain credentials";
     }
   }else{
      //error file not open
      //Serial.println("Failed to open the wifiCrdt.txt file for reading");
      userCredentials = "Failed to open the wifiCrdt.txt file for reading";
    }    
    return valid;
}

void initHotspot(){
// these are the LEMO hotSpot login credentials
const char* ssid = "LEMO";
const char* password = "123456789";
//the file that contains html page to request credentials
File dialogHtml;
//WiFi.softAP(ssid, password);
WiFi.softAP(ssid);
IPAddress IP = WiFi.softAPIP();
Serial.print("AP IP address: ");
Serial.println(IP); 
//server.begin();
hotSpot.begin();
}

void initWebsoc(){
//init as webSoc
 Serial.println("init webSoc: ssid > "+String(userSsid)+" password > "+String(userPassword));
 // Connect to Wi-Fi
 WiFi.begin(userSsid.c_str(),userPassword.c_str());
 while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    int st = WiFi.status();
    Serial.println("Connecting to WiFi.."+String(st));
    Serial.println(WiFi.status());
   
 }
 // Print ESP Local IP Address
 Serial.println(WiFi.localIP());
 ws.onEvent(onEvent);
 webSoc.addHandler(&ws);
 //webSoc.addHandler(&wss); //try to secure
 // Route for root / web page
 webSoc.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
 request->send_P(200, "text/html", index_html, processor);
   });
// Start server
   webSoc.begin();
}

void serveCrdtPage(){
   //there is a new client
    Serial.println("New Client.");
    String currentLine = ""; 
    while (hotSpotClient.connected()) {
      // loop while the client's connected
      Serial.println("client's connected");
      if (hotSpotClient.available()) {
        Serial.println("hotSpot.available");// if there's bytes to read from the client,
        char c = hotSpotClient.read();             // read a byte, then
        //Serial.write(c);                  // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          //the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            hotSpotClient.println("HTTP/1.1 200 OK");
            hotSpotClient.println("Content-type:text/html");
            hotSpotClient.println("Connection: close");
            hotSpotClient.println();
                      
            //this is serving the dialog html to get credentials
            dialogHtml = SPIFFS.open("/index.html");
            while(dialogHtml.available()){
              hotSpotClient.write(dialogHtml.read());
            }
            dialogHtml.close();
            
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            //Serial.println(currentLine);
            //get SSID and password
            int sI = currentLine.indexOf("SSID");
            int hL = currentLine.length();
            if (currentLine.indexOf("SSID") >= 0) {
              String wifiCrd = currentLine.substring(sI,hL);
              //found the SSID and psw
              //Serial.println("wifi crd ***> "+wifiCrd);
              //write the credentials to file(spiffs)
              File ssidPsw = SPIFFS.open("/wifiCrdt.txt", FILE_WRITE);
              if(ssidPsw){
                //file is open for writing          
                if(ssidPsw.print(wifiCrd)){
                  //credentials was writen to file
                  ssidPsw.close();
                  Serial.println ("wifi credentials was writen to file "+wifiCrd);
                  flagSwitchWebsoc = true;
                  WiFi.mode(WIFI_OFF);
                  hotSpot.stop();
                  Serial.println("hotSpot was stopped!!!");
                  flagCredentialDialogServed = true;
                  //setup();
                  bool validCrd = getCredentials();
                  if(validCrd){
                    //the credentials are valid and saved in var                  
                    initWebsoc();
                  }
                } else {
                  Serial.println("File write failed");
                  ssidPsw.close();
                }    
              } else {
                //fail to open
                Serial.println("File failed to open");
              }
            }
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine       
        }
      }
    }
  
    // Clear the header variable
    header = "";
    // Close the connection
    hotSpotClient.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
}
/*-------------end of new command interpreter----------*/
