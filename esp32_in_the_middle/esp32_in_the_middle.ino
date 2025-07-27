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
} SensorData;

SensorData sensorNANO;
char rx_user_buffer[12];
char tx_user_buffer[256]; // Safe buffer for GUI data
char rx_stm_buffer[50];
char tx_stm_buffer[12] = "ABC";
char tx_nano_buffer[53] = "ABC";

bool new_sensor_data_NANO = false;
unsigned long int prevSTM = 0;
unsigned long int prevToNANO = 0;
unsigned long int prevToSTM = 0;

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
  receiveData2(); 

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
  
  if(millis() - prevToSTM >= 100)
  {
    prevToSTM = millis();
    for(int i = 0; i < sizeof(tx_stm_buffer); i++)
    {
        Serial2.write(tx_stm_buffer[i]);
    }
  }
}

void handleWiFiConnection() {
    static unsigned long lastCheck = 0;
    static int attempts = 0;
    if (millis() - lastCheck < 500 && wifiState == CONNECTING) return;
    lastCheck = millis();
    switch (wifiState) {
        case INIT: break;
        case CONNECTING:
            if (WiFi.status() == WL_CONNECTED) {
                wifiState = CONNECTED;
                server.begin();
            } else {
                attempts++;
                if (attempts >= 20) wifiState = FAILED;
            }
            break;
        case CONNECTED: break;
        case FAILED: break;
    }
}

void handleServer() {
    if (wifiState != CONNECTED) return;
    if (!currentClient) {
        currentClient = server.available();
    }
    if (currentClient && !currentClient.connected()) {
        currentClient.stop();
        currentClient = WiFiClient();
    }
    if (currentClient.available() >= sizeof(rx_user_buffer)) {
        for(int i = 0; i < sizeof(rx_user_buffer); i++) {
            rx_user_buffer[i] = currentClient.read();
        }
        if (rx_user_buffer[0] == 'A' && rx_user_buffer[1] == 'B' && rx_user_buffer[2] == 'C') {
            memcpy(&depth_setpoint, rx_user_buffer + 3, 4);
            memcpy(&duration_min, rx_user_buffer + 7, 4);
            memcpy(&send_to_user, rx_user_buffer + 11, 1);
            
            // --- FIX ---
            // This immediately puts the command in the buffer to be sent to the Nano.
            memcpy(tx_nano_buffer + 45, &send_to_user, 1);

            memcpy(tx_stm_buffer + 3, &depth_setpoint, 4);
            memcpy(tx_stm_buffer + 7, &duration_min, 4);
        }
    }
    if (send_to_user) {
        handleTest();
    }
}

void handleTest() {
  if(new_sensor_data_NANO) {
    if (currentClient && currentClient.connected()) {
        currentClient.write((uint8_t*)tx_user_buffer, strlen(tx_user_buffer));
    }
    new_sensor_data_NANO = false;
  }
}

void receiveData1() { //--- From STM
    if(Serial2.available()) {
        static int receive_status;
        static int header_status;
        char temp;
        if(header_status == 0) {
            if(Serial2.available() >= sizeof(rx_stm_buffer) + 3) {
                for(int i = 0; i < sizeof(rx_stm_buffer) + 3; i++) {
                    temp = Serial2.read();
                    if(receive_status == 0 && temp == 'A') receive_status++;
                    else if(receive_status == 1 && temp == 'B') receive_status++;
                    else if(receive_status == 2 && temp == 'C') {
                        receive_status = 0;
                        header_status = 1;
                        break;
                    }
                }
            }
        }
        if(header_status == 1) {
            char real_data[sizeof(rx_stm_buffer)];
            if(Serial2.available() >= sizeof(rx_stm_buffer)) {
                for(int i = 0; i < sizeof(rx_stm_buffer); i++) {
                    real_data[i] = Serial2.read();
                }
                memcpy(rx_stm_buffer, real_data, sizeof(rx_stm_buffer));
                memcpy(tx_nano_buffer + 3, rx_stm_buffer, sizeof(rx_stm_buffer));
                
                // --- FIX ---
                // This ensures the command is re-inserted every time new sensor data arrives.
                memcpy(tx_nano_buffer + 45, &send_to_user, 1);
                
                header_status = 0;
            }
        }
    }
}

// --- FINAL CORRECTED FUNCTION ---
// Replace the entire receiveData2() function in your ESP32 with this.
void receiveData2() { //--- From Nano
    while (Serial.available() > 0) {
        static char line_buffer[200]; 
        static int pos = 0;

        char c = Serial.read();

        // Read characters until we get a newline or the buffer is full
        if (c != '\n' && pos < sizeof(line_buffer) - 1) {
            if (c != '\r') { // Ignore carriage return characters
                line_buffer[pos++] = c;
            }
        } else {
            // We have a full line, so process it
            line_buffer[pos] = '\0'; // Null-terminate the string

            // Attempt to parse the line
            int parsed_items = sscanf(line_buffer, "%hu:%hhu:%hhu:%hu;%hhu;%f;%f;%hhu;%f;%f;%hhu;%f;%f;%hhu;%f;%f;%f;%f;%hhu",
                &sensorNANO.hour, &sensorNANO.minute, &sensorNANO.second, &sensorNANO.millisecond,
                &sensorNANO.ph_ready, &sensorNANO.ph_value, &sensorNANO.ph_interval,
                &sensorNANO.temp_ready, &sensorNANO.temp_value, &sensorNANO.temp_interval,
                &sensorNANO.do_ready, &sensorNANO.do_value, &sensorNANO.do_interval,
                &sensorNANO.turb_ready, &sensorNANO.turb_value, &sensorNANO.turb_interval,
                &sensorNANO.current, &sensorNANO.voltage,
                &sd_card_finished);

            // Only format and send if parsing was successful (sscanf returns the number of items filled)
            if (parsed_items >= 18) {
                // Safely builds the string for the GUI using the parsed data
                snprintf(tx_user_buffer, sizeof(tx_user_buffer),
                         "%u:%u:%u:%u;%u;%.2f;%.2f;%u;%.2f;%.2f;%u;%.2f;%.2f;%u;%.2f;%.2f;%.2f;%.2f;%d\n",
                         sensorNANO.hour, sensorNANO.minute, sensorNANO.second, sensorNANO.millisecond,
                         sensorNANO.ph_ready, sensorNANO.ph_value, sensorNANO.ph_interval,
                         sensorNANO.temp_ready, sensorNANO.temp_value, sensorNANO.temp_interval,
                         sensorNANO.do_ready, sensorNANO.do_value, sensorNANO.do_interval,
                         sensorNANO.turb_ready, sensorNANO.turb_value, sensorNANO.turb_interval,
                         sensorNANO.current, sensorNANO.voltage,
                         sd_card_finished);
                
                new_sensor_data_NANO = true;
            }
            
            pos = 0; // Reset buffer for the next line
        }
    }
}
