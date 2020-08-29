#include <fs.h>
#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BlynkSimpleEsp32.h>
#include <EEPROM.h>

#include "my_auth.h"

#define IN_CIRCUIT

//////////////////////////// One Wire Stuff (Temperature)/////////////////////////////////////////////


// Data wire is plugged into pin D6
#define ONE_WIRE_BUS 21

// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

float raw_temps_0[10] = {0,0,0,0,0,0,0,0,0,0};
float raw_temps_1[10] = {0,0,0,0,0,0,0,0,0,0};
float raw_temps_2[10] = {0,0,0,0,0,0,0,0,0,0};
float raw_temps_3[10] = {0,0,0,0,0,0,0,0,0,0};

float avg_temp_0 = 0.0;
float avg_temp_1 = 0.0;
float avg_temp_2 = 0.0;
float avg_temp_3 = 0.0;

float temp_0_correction = 0.0;
float temp_1_correction = 0.0;
float temp_2_correction = 0.0;
float temp_3_correction = 0.0;

int rt_c = 0;

float avg_temp_supply = 0.0; //enthalpy supply
float avg_temp_coil = 0.0; //coil 
float avg_temp_supply2 = 0.0; //supply 2
float avg_temp_return = 0.0; //enthalpy return

long last_temp_read_time = 0;
int read_temp_interval = 200; //(every 100 ms)

void update_temp_readings(void)
{
  long now = millis();
  if (abs(now - last_temp_read_time) > read_temp_interval)
  {
    sensors.requestTemperatures(); // Send the command to get temperature reading

    last_temp_read_time = now;

    //first ds18b20 - 0 
    float temp = sensors.getTempCByIndex(0); //enthalpy supply
    if (temp == -127.0) temp = avg_temp_supply;
    else temp = temp * (9.0/5.0) + 32; //to fahrenheit
    
    raw_temps_0[rt_c] = temp + temp_0_correction;

    float sum = 0.0;
    for (int i=0;i<10;i++)
    {
      sum+=raw_temps_0[i];
    }
    avg_temp_0 = sum/10.0;
    avg_temp_supply = avg_temp_0;

    //second ds18b20 - 1
    float temp2 = sensors.getTempCByIndex(1); // coil
    if (temp2 == -127.0) temp2 = avg_temp_coil;
    else temp2 = temp2 * (9.0/5.0) + 32; //to fahrenheit
    
    raw_temps_1[rt_c] = temp2 + temp_1_correction;

    float sum2 = 0.0;
    for (int i=0;i<10;i++)
    {
      sum2+=raw_temps_1[i];
    }
    avg_temp_1 = sum2/10.0;
    avg_temp_coil = avg_temp_1;

    //third ds18b20 - 2
    float temp3 = sensors.getTempCByIndex(2); //supply 2
    if (temp3 == -127.0) temp3 = avg_temp_supply2;
    else temp3 = temp3 * (9.0/5.0) + 32; //to fahrenheit
    
    raw_temps_2[rt_c] = temp3 + temp_2_correction;

    float sum3 = 0.0;
    for (int i=0;i<10;i++)
    {
      sum3+=raw_temps_2[i];
    }
    avg_temp_2 = sum3/10.0;
    avg_temp_supply2 = avg_temp_2;

    //fourth ds18b20 - 3
    float temp4 = sensors.getTempCByIndex(3); //enthalpy return
    if (temp4 == -127.0) temp4 = avg_temp_supply2;
    else temp4 = temp4 * (9.0/5.0) + 32; //to fahrenheit
    
    raw_temps_3[rt_c] = temp4 + temp_3_correction;

    float sum4 = 0.0;
    for (int i=0;i<10;i++)
    {
      sum4+=raw_temps_3[i];
    }
    avg_temp_3 = sum4/10.0;
    avg_temp_return = avg_temp_3;



    if (rt_c == 9) rt_c=0;
    else rt_c++;
  }
  
}


/////////////////////////////////////////////Humidity Stuff (BME280)//////////////////////////////////
void update_humidity_readings();

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C //return
Adafruit_BME280 bme2;       //supply

float raw_humidity_0[10] = {0,0,0,0,0,0,0,0,0,0}; //return
float raw_pressure_0[10] = {0,0,0,0,0,0,0,0,0,0};
float raw_bme_temp_0[10] = {0,0,0,0,0,0,0,0,0,0};
int rh_c = 0;

float avg_humidity_0 = 0.0;
float avg_pressure_0 = 0.0;
float avg_bme_temp_0 = 0.0;

float humidity_correction_0 = 0.0;
float pressure_correction_0 = 0.0;
float bme_temp_correction_0 = 0.0;

float avg_humidity_return = 0.0; //enthalpy return
float avg_pressure_return = 0.0;
float avg_bme_temp_return = 0.0;

float raw_humidity_1[10] = {0,0,0,0,0,0,0,0,0,0};
float raw_pressure_1[10] = {0,0,0,0,0,0,0,0,0,0};
float raw_bme_temp_1[10] = {0,0,0,0,0,0,0,0,0,0};
int rh_c2 = 0;

float avg_humidity_1 = 0.0;
float avg_pressure_1 = 0.0;
float avg_bme_temp_1 = 0.0;

float humidity_correction_1 = 0.0;
float pressure_correction_1 = 0.0;
float bme_temp_correction_1 = 0.0;

float avg_humidity_supply = 0.0; //supply
float avg_pressure_supply = 0.0;
float avg_bme_temp_supply = 0.0;



long last_humidity_read_time = 0;
int read_humidity_interval = 200; //(every 100 ms)

void update_humidity_readings()
{
  long now = millis();

  if (abs(now - last_humidity_read_time) > read_humidity_interval)
  {
    last_humidity_read_time = now;
    
    //first bme measurements
    bme.takeForcedMeasurement(); // has no effect in normal mode
    float humidity = bme.readHumidity();//%
    float pressure = (bme.readPressure() / 100.0F) * .03;//x/100 to hpa * .03 to inHG
    float temp = bme.readTemperature();//*C

    raw_humidity_0[rh_c] = humidity + humidity_correction_0;
    raw_pressure_0[rh_c] = pressure + pressure_correction_0;
    raw_bme_temp_0[rh_c] = (temp * (9.0/5.0) + 32) + bme_temp_correction_0;

    float num_buckets = 3.0;

    float hsum=0.0,psum=0.0,tsum=0.0;
    for (int i=0;i<int(num_buckets);i++)
    {
      hsum+=raw_humidity_0[i];
      psum+=raw_pressure_0[i];
      tsum+=raw_bme_temp_0[i];
    }

    avg_humidity_0 = hsum/num_buckets;
    avg_humidity_return = avg_humidity_0;

    avg_pressure_0 = psum/num_buckets;
    avg_pressure_return = avg_pressure_0; 

    avg_bme_temp_0 = tsum/num_buckets;
    avg_bme_temp_return = avg_bme_temp_0;


    //second bme measurements
    bme2.takeForcedMeasurement(); // has no effect in normal mode
    float humidity2 = bme2.readHumidity() + 1.0;// difference in sensor correction...
    float pressure2 = (bme2.readPressure() / 100.0F) * .03;//x/100 to hpa * .03 to inHG
    float temp2 = bme2.readTemperature();//*C.

    //Serial.printf("h2 %f p2 %f t2 %f\n", humidity2,pressure2,temp2);
    raw_humidity_1[rh_c] = humidity2 + humidity_correction_1;
    raw_pressure_1[rh_c] = pressure2 + pressure_correction_1;
    raw_bme_temp_1[rh_c] = (temp2 * (9.0/5.0) + 32) + bme_temp_correction_1;

    float hsum2=0.0,psum2=0.0,tsum2=0.0;
    for (int i=0;i<int(num_buckets);i++)
    {
      hsum2+=raw_humidity_1[i];
      psum2+=raw_pressure_1[i];
      tsum2+=raw_bme_temp_1[i];
    }

    avg_humidity_1 = hsum2/num_buckets;
    avg_humidity_supply = avg_humidity_1;

    avg_pressure_1 = psum2/num_buckets;
    avg_pressure_supply = avg_pressure_1;

    avg_bme_temp_1 = tsum2/num_buckets;
    avg_bme_temp_supply = avg_bme_temp_1;


    if (rh_c==(int(num_buckets)-1)) rh_c=0;
    else rh_c++;
  }
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////Enthalpy Stuff///////////////////////////////////////////////////////////

float raw_enthalpy_supply[10] = {0,0,0,0,0,0,0,0,0,0};
int e_c = 0;
float avg_enthalpy_supply = 0.0;

float raw_enthalpy_return[10] = {0,0,0,0,0,0,0,0,0,0};
float avg_enthalpy_return = 0.0;

unsigned long last_enthalpy_read_time = 0;
int read_enthalpy_interval = 1000; //(every 100 ms)

float estimated_btu_hr = 0;
float estimated_airflow = 1241.0;//1375.0; //cfm
float estimated_sensible_cooling = 0.0;
float estimated_latent_cooling = 0.0;

void update_btu_hr(void)
{
  float instant_btu_hr = estimated_airflow * (avg_enthalpy_return-avg_enthalpy_supply) * 4.5;

  if (instant_btu_hr < 0.0)
  {
    estimated_btu_hr = 0.0;
  }
  else estimated_btu_hr = instant_btu_hr;
}

float calc_mixing_ratio(float temperature, float pressure, float humidity)
{
  //https://www.aqua-calc.com/calculate/humidity
  
  float current_temp_k = temperature + 273.15;

  float current_pressure_MPa = pressure/295.29983071445; //inHG 2 MPa
  
  float tcrit = 647.096;//kelvin
  float pcrit = 22.064;//MPa

  float theta = 1 - (current_temp_k / tcrit);

  float e = 2.718281828459;
  // Tc ⁄ T × (a1 × ϑ + a2 × ϑ1.5+ a3 × ϑ3+ a4 × ϑ3.5+ a5 × ϑ4+ a6 × ϑ7.5)
  //Serial.printf("ck %f theta %f\n", current_temp_k, theta);


  float aa = (-7.85951783 * theta);
  float bb = (1.84408259 * pow(theta,1.5));
  float cc = (-11.7866497  * pow(theta,3.0));
  float dd = (22.6807411 * pow(theta,3.5));
  float ee = (-15.9618719 * pow(theta,4));
  float ff = (1.80122502 * pow(theta,7.5));

  float right_side = 647.096 / current_temp_k * (aa + bb + cc + dd + ee + ff);
  
  //float right_side=10;
  float Pws = pow(e,right_side) * pcrit; //saturated water vapor pressure (MPa)
  
  //float saturated_mixing_ratio = 621.97 * Pws / (current_pressure_MPa - Pws);//g/kg

  float Pw = humidity * Pws / 100.0; //water vapor pressure

  float W = 621.97 * Pw / (current_pressure_MPa-Pw); //mixing ratio or humidity ratio g/kG

  return W;
}

float calc_enthalpy(float temperature, float pressure, float humidity)
{

    float current_temp_f = temperature;
    float current_temp_c = (current_temp_f-32.0)/1.80;
  
    float W = calc_mixing_ratio(current_temp_c, pressure, humidity);
    float enthalpy = current_temp_c * (1.01 + 0.00189 * W) + 2.5 * W; //kJ/kg

    float enthalpy_imperial = enthalpy * 0.429923;

    return enthalpy_imperial;
}

void update_enthalpy(void)
{
  unsigned long now = millis();

  if (abs(now - last_enthalpy_read_time) > read_enthalpy_interval)
  {
    //add an element to the enthalpy circular buffer
    float avged_temp_supply = (avg_temp_supply + avg_temp_supply2 + avg_bme_temp_1)/3.0;
    raw_enthalpy_supply[e_c] = calc_enthalpy(avged_temp_supply, avg_pressure_supply,avg_humidity_supply); //supply
    raw_enthalpy_return[e_c] = calc_enthalpy(avg_temp_return, avg_pressure_return, avg_humidity_return); //return

    float supply_c = (avged_temp_supply-32.0)*(5.0/9.0);
    float return_c = (avg_temp_return-32.0)*(5.0/9.0);
    //sensible
    estimated_sensible_cooling = (1.006 * 1.18265 * (estimated_airflow/2118.8799727597) * (return_c - supply_c)) * 3412.142; //mult by 3412 to get btu instead of kw

    //latent
    float w2 = calc_mixing_ratio(supply_c, avg_pressure_supply, avg_humidity_supply)/1000.0; //g/kg to kg/kg or lb/lb
    float w1 = calc_mixing_ratio(return_c, avg_pressure_return, avg_humidity_return)/1000.0; //g/kg to kg/kg or lb/lb

    //https://youtu.be/6Kg5gH7OiOs?t=712
    estimated_latent_cooling = 4890 * estimated_airflow * (w1-w2);

    //calculate the new average in the circular buffferss
    float num_buckets = 10.0;

    float esum=0.0,esum2=0.0;
    for (int i=0;i<int(num_buckets);i++)
    {
      esum+=raw_enthalpy_supply[i];
      esum2+=raw_enthalpy_return[i];
    }
    //place the result in their respective variables
    avg_enthalpy_supply = esum/num_buckets;
    avg_enthalpy_return = esum2/num_buckets;

    update_btu_hr();
    
    //manage circular buffer 
    if (e_c==(int(num_buckets)-1)) e_c=0;
    else e_c++;



    //update timestamp
    last_enthalpy_read_time = now;
  }

}

//sensible heat kw = specific_heat_air(1.006) * density(1.18265) * (flow_rate/(cfm->m3/s) const) * delt(c);
//https://www.engineeringtoolbox.com/cooling-heating-equations-d_747.html

///////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////Hearbeat Stuff//////////////////////////////////////////////////
unsigned long last_led_on_time = 0;
unsigned long last_led_off_time = 0;
uint8_t led_state = 0; 
uint8_t led_pin = 2;

void myheartbeat(void)
{
  unsigned long now = millis();
  if (led_state)
  {
    if (abs(now - last_led_on_time) > 100)
    {
      digitalWrite(led_pin, LOW);
      led_state=0;
      last_led_off_time = now;
    } 
  }
  else
  {
    if (abs(now - last_led_off_time) > 1900)
    {
      digitalWrite(led_pin, HIGH);
      led_state=1;
      last_led_on_time = now;
    }
  }
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////


void zero_all(void)
{

  //calc up all the average params...
  float temp_sum = avg_temp_0 + avg_temp_1 + avg_temp_2 + avg_temp_3 + avg_bme_temp_0 + avg_bme_temp_1;

  float avg_temp = temp_sum / 6.0;

  float humidity_sum = avg_humidity_0 + avg_humidity_1;
  float avg_humidity = humidity_sum / 2.0;

  float pressure_sum = avg_pressure_0 + avg_pressure_1;
  float avg_pressure = pressure_sum / 2.0;

  //then apply the corrections

  temp_0_correction = avg_temp - avg_temp_0;
  temp_1_correction = avg_temp - avg_temp_1;
  temp_2_correction = avg_temp - avg_temp_2;
  temp_3_correction = avg_temp - avg_temp_3;

  bme_temp_correction_0 = avg_temp - avg_bme_temp_0;
  bme_temp_correction_1 = avg_temp - avg_bme_temp_1;

  humidity_correction_0 = avg_humidity - avg_humidity_0;
  humidity_correction_1 = avg_humidity - avg_humidity_1;

  pressure_correction_0 = avg_pressure - avg_pressure_0;
  pressure_correction_1 = avg_pressure - avg_pressure_1;

}

//////////////////////////////////////////////EEPROM Stuff/////////////////////////////////////////////

void load_from_EEPROM(void)
{
  temp_0_correction = EEPROM.readFloat(0);
  temp_1_correction = EEPROM.readFloat(4);
  temp_2_correction = EEPROM.readFloat(8);
  temp_3_correction = EEPROM.readFloat(12);

  bme_temp_correction_0 = EEPROM.readFloat(16);
  bme_temp_correction_1 = EEPROM.readFloat(20);

  humidity_correction_0 = EEPROM.readFloat(24);
  humidity_correction_1 = EEPROM.readFloat(28);

  pressure_correction_0 = EEPROM.readFloat(32);
  pressure_correction_1 = EEPROM.readFloat(36); 
}

void save_to_EEPROM(void)
{
  EEPROM.writeFloat(0,temp_0_correction);
  EEPROM.commit();
  EEPROM.writeFloat(4,temp_1_correction);
  EEPROM.commit();
  EEPROM.writeFloat(8,temp_2_correction);
  EEPROM.commit();
  EEPROM.writeFloat(12,temp_3_correction);
  EEPROM.commit();

  EEPROM.writeFloat(16,bme_temp_correction_0);
  EEPROM.commit();
  EEPROM.writeFloat(20,bme_temp_correction_1);
  EEPROM.commit();

  EEPROM.writeFloat(24,humidity_correction_0);
  EEPROM.commit();
  EEPROM.writeFloat(28,humidity_correction_1);
  EEPROM.commit();

  EEPROM.writeFloat(32,pressure_correction_0);
  EEPROM.commit();
  EEPROM.writeFloat(36,pressure_correction_1);
  EEPROM.commit();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////MQTT Stuff//////////////////////////////////////////////////////////

int update_interval=1000;

String area = "utilities";
String sensor = "air_handler_enthalpy_sensor";
const char* mqtt_server = "homeassistant";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(512)
char msg[MSG_BUFFER_SIZE];

long last_publish_time = 0;
long last_reconnect_attempt = 0;


void reconnect(void) {
  // Loop until we're reconnected
  if (!client.connected() && (millis() - last_reconnect_attempt > 5000)) {
    Serial.print("Attempting MQTT connection...\n");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    last_reconnect_attempt = millis();
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_pass)) {
      Serial.println("connected");

      String str = area + "/" + sensor + "/calibrate";
      boolean res = client.subscribe(str.c_str());
      if (res)
      {
        Serial.print("Successfully subscribed to ");
        Serial.println(str);
      }
    }
    else
    {
      Serial.println("Connection to MQTT failed\n");
    }
    
  }
}

void publish_to_topic(String topic, String value)
{
  const char* top = topic.c_str();
  const char * val_string = value.c_str();
  client.publish(top, val_string);
}

void publish_readings(void)
{
  long now = millis();

  if (abs(now - last_publish_time) > update_interval)
  {
    //return side values
    publish_to_topic(String(area + "/" + sensor + "/temperature_return"),String(avg_temp_return,2));

    publish_to_topic(String(area + "/" + sensor + "/humidity_return"),String(avg_humidity_return,2));

    publish_to_topic(String(area + "/" + sensor + "/pressure_return"),String(avg_pressure_return,2));

    publish_to_topic(String(area + "/" + sensor + "/enthalpy_return"),String(avg_enthalpy_return,2));

    //supply side values
    publish_to_topic(String(area + "/" + sensor + "/temperature_supply"),String(((avg_temp_supply+avg_temp_supply2+avg_bme_temp_1)/3.0),2));

    publish_to_topic(String(area + "/" + sensor + "/temperature_supply1"),String(avg_temp_supply,2));
    
    publish_to_topic(String(area + "/" + sensor + "/temperature_supply2"),String(avg_temp_supply2,2));

    publish_to_topic(String(area + "/" + sensor + "/humidity_supply"),String(avg_humidity_supply,2));

    publish_to_topic(String(area + "/" + sensor + "/pressure_supply"),String(avg_pressure_supply,2));

    publish_to_topic(String(area + "/" + sensor + "/enthalpy_supply"),String(avg_enthalpy_supply,2));

    publish_to_topic(String(area + "/" + sensor + "/evap_coil_temp"),String(avg_temp_coil,2));

    //etc
    publish_to_topic(String(area + "/" + sensor + "/estimated_btu_hr"),String(estimated_btu_hr,2));

    publish_to_topic(String(area + "/" + sensor + "/estimated_sensible_cooling"),String(estimated_sensible_cooling,2));

    publish_to_topic(String(area + "/" + sensor + "/estimated_latent_cooling"),String(estimated_latent_cooling,2));

    publish_to_topic(String(area + "/" + sensor + "/now"),String(now));

    publish_to_topic(String(area + "/" + sensor + "/last_bme280_update"),String(last_humidity_read_time));

    publish_to_topic(String(area + "/" + sensor + "/last_ds18b20_update"),String(last_temp_read_time));

    publish_to_topic(String(area + "/" + sensor + "/bme_return_temp"),String(avg_bme_temp_0,2));

    publish_to_topic(String(area + "/" + sensor + "/bme_supply_temp"),String(avg_bme_temp_1,2));

    last_publish_time = now;
  }
}

void callback(char* topic, byte* payload, unsigned int length)
{
  char t[512];
  strcpy(t,topic);

  byte b[512];

  for (int i=0; i<length;i++)
  {
    b[i] = payload[i];
  }
  
  long now = millis();
  publish_to_topic(String(area + "/" + sensor + "/last_mqtt_request"),String(now));
  
  if (length < MSG_BUFFER_SIZE)
  {
    int counter = 0;
    boolean done = false;
    boolean success = false;

    char * token = strtok(t, "/");
    // loop through the string to extract all other tokens
    while( token != NULL && done == false) 
    {
        Serial.printf( " %s\n", token ); //printing each token
        //publish_to_topic(String(area + "/" + sensor + "/tokens"),String(token));
        switch (counter)
        {
          case 0:
          {
            int res = strcmp(area.c_str(), token);
            if (res != 0)
            {
              done = true;
              //publish_to_topic(String(area + "/" + sensor + "/error"),String("failed to parse area" + area + String(token)));
              //Serial.printf("Failed to parse area |%s| |%s|\n",area, token);
            }
            
          }
          break;
          case 1:
          {
            int res = strcmp(sensor.c_str(), token);
            if (res != 0) 
            {
              done = true;
              //publish_to_topic(String(area + "/" + sensor + "/error"),String("failed to parse sensor"));
              //Serial.printf("Failed to parse sensor |%s| |%s|\n",sensor, token);

            }
          }
          break;
          case 2:
          {
            char s[] = "calibrate";
            int res = strcmp(s, token);
            if (res != 0) 
            {
              done = true;
              //publish_to_topic(String(area + "/" + sensor + "/error"),String("failed to parse calibrate"));
              //Serial.printf("Failed to parse calibrate |%s|\n", token);

            }
            else
            {
              char s[] = "all";
              char r[] = "save";
              if (strcmp(s, (char*)b) == 0)
              {
                zero_all();
                publish_to_topic(String(area + "/" + sensor + "/last_calibration"),String(millis()));
                //Serial.printf("Calibrated...\n");
                done=true;
              }
              else if (strcmp(r, (char*)b) == 0)
              {
                save_to_EEPROM();
                done=true;
              }
            }
          }
          break;
          default: done=true;
          break;
        }
        counter++;
        token = strtok(NULL, "/");
    }
    }
    else
    {
      String size = String(length);
      publish_to_topic(String(area + "/" + sensor + "/error"),String("msg_too_big " + size));
    }
    
    

  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
long last_blynk_time = 0;
long blynk_update_interval = 2000;

void update_blynk(void)
{
  long now = millis();

  if (now - last_blynk_time > blynk_update_interval)
  {
    Blynk.virtualWrite(V1,avg_temp_coil);//coil
    Blynk.virtualWrite(V0,avg_temp_supply); //supply
    Blynk.virtualWrite(V3,avg_humidity_return); //return hum
    Blynk.virtualWrite(V4,avg_enthalpy_return); //return enthalpy
    Blynk.virtualWrite(V5,avg_enthalpy_supply); //supply enthalpy
    Blynk.virtualWrite(V6, avg_temp_return);
    Blynk.virtualWrite(V7, estimated_btu_hr);
    last_blynk_time = now;
  }
  
}




//////////////////////////////////////////////Blynk Stuff/////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(led_pin, OUTPUT);
  
  ////////////////////////////////WiFi Manager/////////////////////////////////////////////////////////
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

  //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wm;

  //reset settings - wipe credentials for testing
  //wm.resetSettings();

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
  // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
  // then goes into a blocking loop awaiting configuration and will return success result
  
  bool res;
  wm.setConfigPortalTimeout(180);

  res = wm.autoConnect(wifimanager_ssid,wifimanager_pass); // password protected ap
  if(!res) {  
    ESP.restart();
  } 
  ///////////////////////////////////////////////////////////////////////////////////////////////////////





  //////////////////////////////////// O T A ////////////////////////////////////////////////////////////
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("air-handler-enthalpy");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  /////////////////////////////////////////////////////////////////////////////////////////////////////////



  ////////////////////////////////////////MQTT/////////////////////////////////////////////////////////////
  //Serial.println(MDNS.queryHost(mqtt_server).toString().c_str());
  client.setServer(mqtt_server, 1883);
  client.setBufferSize(1024);
  client.setCallback(callback);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////


  #ifdef IN_CIRCUIT
  ////////////////////////////////////////DS18B20//////////////////////////////////////////////////////////
  sensors.begin(); 
  /////////////////////////////////////////////////////////////////////////////////////////////////////////



  ////////////////////////////////////////B M E 2 8 0 /////////////////////////////////////////////////////
  Wire.begin(23,22);
  if (! bme.begin(0x77, &Wire)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  if (! bme2.begin(0x76, &Wire)) {
    Serial.println("Could not find a valid BME280-2 sensor, check wiring!");
    while (1);
  }

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );

  bme2.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X16, // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X16, // humidity
                    Adafruit_BME280::FILTER_OFF   );
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  #endif


  //////////////////////////////////////B L Y N K//////////////////////////////////////////////////////////
  Blynk.config(blynk_auth);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////

  EEPROM.begin(64);

  load_from_EEPROM();
}

void loop() {
  long start = millis();
  // put your main code here, to run repeatedly:
  ArduinoOTA.handle();
  if (!client.connected()) reconnect();
  #ifdef IN_CIRCUIT
  update_temp_readings();
  update_humidity_readings();
  update_enthalpy();
  #endif
  publish_readings();
  update_blynk();
  myheartbeat();
  Blynk.run();

  client.loop();

  long duration = millis() - start;
}