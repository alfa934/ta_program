#include <WiFi.h>

const char* ssid = "titanium";
const char* password = "titanium";

// Static IP configuration
IPAddress local_IP(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiServer server(80);

// State machine variables
enum ConnectionState { INIT, CONNECTING, CONNECTED, FAILED };
ConnectionState wifiState = INIT;

// Client handling variables
WiFiClient currentClient;
unsigned long clientConnectTime = 0;

unsigned long testStartTime = 0;
unsigned long testDuration = 0;
unsigned long lastDataTime = 0;
String testDepth = "";
String testDurationStr = "";

int32_t depth_setpoint = -1;
int32_t duration_min = -1;
uint8_t send_to_user = 0;
uint8_t sd_card_finished = 0;

typedef struct
{
  uint16_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t millisecond;
  
  uint8_t ph_ready;
  float ph_value;
  float ph_interval;
  
  uint8_t temp_ready;
  float temp_value;
  float temp_interval;
  
  uint8_t do_ready;
  float do_value;
  float do_interval;
  
  uint8_t turb_ready;
  float turb_value;
  float turb_interval;
  
  float current;
  float voltage;
  
  uint8_t done_state;
} SensorData;

SensorData sensorNANO;
char rx_user_buffer[12];
char tx_user_buffer[55] = "ABC";
char rx_stm_buffer[50];
char tx_stm_buffer[12] = "ABC";
char rx_nano_buffer[52];
char tx_nano_buffer[53] = "ABC";
String tx_str = "";


bool new_sensor_data_STM = false;
bool new_sensor_data_NANO = false;
unsigned long int prevSTM = 0;
unsigned long int prevNANO = 0;
unsigned long int prevToSTM = 0;
unsigned long int prevToNANO = 0;


void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  
  if (!WiFi.config(local_IP, gateway, subnet))
  {
    return;
  }
  
  WiFi.begin(ssid, password);
  wifiState = CONNECTING;
}

void loop()
{
  handleWiFiConnection();
  
  handleServer();

  if(millis() - prevSTM >= 10)
  {
    prevSTM = millis();

    receiveData1();
    
  }

  if(millis() - prevToNANO >= 50)
  {
    for(int i = 0; i < sizeof(tx_nano_buffer); i++)
    {
        Serial.write(tx_nano_buffer[i]);
    }
    prevToNANO = millis();
  }

  if(millis() - prevNANO >= 2)
  {
    prevNANO = millis();
    receiveData2();
  }

  if(millis() - prevToSTM >= 100)
  {
    prevToSTM = millis();
    for(int i = 0; i < sizeof(tx_stm_buffer); i++)
    {
        Serial2.write(tx_stm_buffer[i]);
    }
  }
  
}

void handleWiFiConnection()
{
  static unsigned long lastCheck = 0;
  static int attempts = 0;
  
  if (millis() - lastCheck < 500 && wifiState == CONNECTING) return;
  lastCheck = millis();

  switch (wifiState)
  {
    case INIT:
      break;
      
    case CONNECTING:
      if (WiFi.status() == WL_CONNECTED)
      {
        wifiState = CONNECTED;
        server.begin();
      } 
      else
      {
        attempts++;
        if (attempts >= 20)
        {
          wifiState = FAILED;
        }
      }
      break;
      
    case CONNECTED:
      // Nothing to do here
      break;
      
    case FAILED:
      // Optional: implement recovery logic here
      break;
  }
}

void handleServer()
{
    if (wifiState != CONNECTED) return;
    
    
    if (!currentClient)
    {
        currentClient = server.available();
        if (currentClient)
        {
            return;
        }
    }
    
    if (!currentClient.connected())
    {
        currentClient.stop();
        return;
    }
    
    if (currentClient.available() >= sizeof(rx_user_buffer))
    {
        for(int i = 0; i < sizeof(rx_user_buffer); i++)
        {
            rx_user_buffer[i] = currentClient.read();
        }
        
        if (rx_user_buffer[0] == 'A' && rx_user_buffer[1] == 'B' && rx_user_buffer[2] == 'C')
        {
            memcpy(&depth_setpoint, rx_user_buffer + 3, 4);
            memcpy(&duration_min, rx_user_buffer + 7, 4);
            
            memcpy(&send_to_user, rx_user_buffer + 11, 1);
            memcpy(tx_nano_buffer + 45, &send_to_user, 1);

            memcpy(tx_stm_buffer + 3, &depth_setpoint, 4);
            memcpy(tx_stm_buffer + 7, &duration_min, 4);
        }
    }

  
    if (send_to_user)
    {
        handleTest();
    }
}

void handleTest()
{
  if (!currentClient.connected())
  {
    return;
  }

  if(new_sensor_data_NANO)
  {
    currentClient.write(tx_str.c_str(), tx_str.length());
    new_sensor_data_NANO = false;
  }

}


void receiveData1() //--- STM
{
    if(Serial2.available())
    {
        static int receive_status;
        static int header_status;
        char temp;

        if(header_status == 0)
        {
            if(Serial2.available() >= sizeof(rx_stm_buffer) + 3)
            { 
                for(int i = 0; i < sizeof(rx_stm_buffer) + 3; i++)
                { 
                    temp = Serial2.read();

                    if(receive_status == 0 && temp == 'A')      
                    { 
                        receive_status++; 
                    }
                    else if(receive_status == 1 && temp == 'B') 
                    { 
                        receive_status++; 
                    }
                    else if(receive_status == 2 && temp == 'C')
                    {
                        receive_status = 0;
                        header_status = 1;
                        break;
                    }
                }
            }
        }
        
        if(header_status == 1)
        { 
            char real_data[sizeof(rx_stm_buffer)];
            if(Serial2.available() >= sizeof(rx_stm_buffer))
            {
                for(int i = 0; i < sizeof(rx_stm_buffer); i++)
                {
                    real_data[i] = Serial2.read();
                }

                memcpy(rx_stm_buffer, real_data, sizeof(rx_stm_buffer));

                memcpy(tx_nano_buffer + 3, rx_stm_buffer, sizeof(rx_stm_buffer)); 
                
                header_status = 0;
            }
        }
    }
}

void receiveData2() //--- nano
{
    if(Serial.available())
    {
        static int receive_status;
        static int header_status;
        char temp;

        if(header_status == 0)
        {
            if(Serial.available() >= sizeof(rx_nano_buffer) + 3)
            { 
                for(int i = 0; i < sizeof(rx_nano_buffer) + 3; i++)
                { 
                    temp = Serial.read();

                    if(receive_status == 0 && temp == 'A')      
                    { 
                        receive_status++; 
                    }
                    else if(receive_status == 1 && temp == 'B') 
                    { 
                        receive_status++; 
                    }
                    else if(receive_status == 2 && temp == 'C')
                    {
                        receive_status = 0;
                        header_status = 1;
                        break;
                    }
                }
            }
        }
        
        if(header_status == 1)
        { 
            char real_data[sizeof(rx_nano_buffer)];
            if(Serial.available() >= sizeof(rx_nano_buffer))
            {
                for(int i = 0; i < sizeof(rx_nano_buffer); i++)
                {
                    real_data[i] = Serial.read();
                }

                memcpy(rx_nano_buffer, real_data, sizeof(rx_nano_buffer));
                
                memcpy(&sensorNANO.hour, rx_nano_buffer, 2);
                memcpy(&sensorNANO.minute, rx_nano_buffer + 2, 1);
                memcpy(&sensorNANO.second, rx_nano_buffer + 3, 1);
                memcpy(&sensorNANO.millisecond, rx_nano_buffer + 4, 2);
        
                memcpy(&sensorNANO.ph_ready, rx_nano_buffer + 6, 1);
                memcpy(&sensorNANO.ph_value, rx_nano_buffer + 7, 4);
                memcpy(&sensorNANO.ph_interval, rx_nano_buffer + 11, 4);
        
                memcpy(&sensorNANO.temp_ready, rx_nano_buffer + 15, 1);
                memcpy(&sensorNANO.temp_value, rx_nano_buffer + 16, 4);
                memcpy(&sensorNANO.temp_interval, rx_nano_buffer + 20, 4);
        
                memcpy(&sensorNANO.do_ready, rx_nano_buffer + 24, 1);
                memcpy(&sensorNANO.do_value, rx_nano_buffer + 25, 4);
                memcpy(&sensorNANO.do_interval, rx_nano_buffer + 29, 4);
        
                memcpy(&sensorNANO.turb_ready, rx_nano_buffer + 33, 1);
                memcpy(&sensorNANO.turb_value, rx_nano_buffer + 34, 4);
                memcpy(&sensorNANO.turb_interval, rx_nano_buffer + 38, 4);
        
                memcpy(&sensorNANO.current, rx_nano_buffer + 42, 4);
                memcpy(&sensorNANO.voltage, rx_nano_buffer + 46, 4);

                memcpy(&sd_card_finished, rx_nano_buffer + 50, 1);
        
                // Build transmission string
                tx_str  = "";
                tx_str += String(sensorNANO.hour); //--- hour
                tx_str += ":";
                tx_str += String(sensorNANO.minute); //--- minute
                tx_str += ":";
                tx_str += String(sensorNANO.second); //--- second
                tx_str += ":";
                tx_str += String(sensorNANO.millisecond); //--- millisecond
                
                tx_str += ";";
                tx_str += String(sensorNANO.ph_ready);
                tx_str += ";";
                tx_str += String(sensorNANO.ph_value);
                tx_str += ";";
                tx_str += String(sensorNANO.ph_interval);
        
                tx_str += ";";
                tx_str += String(sensorNANO.temp_ready);
                tx_str += ";";
                tx_str += String(sensorNANO.temp_value);
                tx_str += ";";
                tx_str += String(sensorNANO.temp_interval);
                
                tx_str += ";";
                tx_str += String(sensorNANO.do_ready);
                tx_str += ";";
                tx_str += String(sensorNANO.do_value);
                tx_str += ";";
                tx_str += String(sensorNANO.do_interval);
                
                tx_str += ";";
                tx_str += String(sensorNANO.turb_ready);
                tx_str += ";";
                tx_str += String(sensorNANO.turb_value);
                tx_str += ";";
                tx_str += String(sensorNANO.turb_interval);
                
                tx_str += ";";
                tx_str += String(sensorNANO.current);
                tx_str += ";";
                tx_str += String(sensorNANO.voltage);
                tx_str += ";";
                tx_str += String(sd_card_finished);
                tx_str += "\n";

                new_sensor_data_NANO = true;
                
                header_status = 0;
            }
        }
    }
}
