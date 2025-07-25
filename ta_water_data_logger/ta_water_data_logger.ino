/*
 * Data Logger Program
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

/*
 * Global Variables
 */
#define chipSelect 4

File openFile;

typedef struct
{
    uint16_t hour_t;
    uint8_t  minute_t;
    uint8_t  second_t;
    uint16_t millisecond_t;
} Time_t;

/* Data for sensors
 * 00) timestamp hour
 * 01) timestamp minute
 * 02) timestamp seconds
 * 03) timestamp milliseconds
 * 04) pH status
 * 05) pH
 * 06) pH interval
 * 07) temperature water status
 * 08) temperature water
 * 09) temperature interval
 * 10) diss. oxygen status
 * 11) diss. oxygen
 * 12) diss. oxygen interval
 * 13) turbidity status
 * 14) turbidity
 * 15) turbidity interval
 * 16) send_state
 * 17) current system
 * 18) voltage system
 * 
 */

char sensor_data_rx[50];     //--- not including ABC
char sensor_data_tx[55] = "ABC";

unsigned long int timeNow    = 0;
unsigned long int timePrev   = 0;
unsigned long int sensorNow  = 0;
unsigned long int sensorPrev = 0;
unsigned long prevTime = 0;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
long int ina_interval = 2000;

Time_t Global_Time;

uint8_t save_ph = 0, save_temp = 0, save_do = 0, save_turb = 0, send_to_user = 0;

float value_ph = 0, value_temp = 0, value_do = 0, value_turb = 0;

float interval_ph = 0, interval_temp = 0, interval_do = 0, interval_turb = 0;


void setup()
{
    Serial.begin(115200);
    pinMode(SS, OUTPUT);
    pinMode(13, OUTPUT);
    
    if (! ina219.begin())
    {
        while (1);
    }
   
    if (!SD.begin(chipSelect))
    {
        return;
    }

    while(!(current_mA && loadvoltage))
    {
        readINA219();
    }
    delay(500);
    writeFiles();
}

unsigned long int test = 0;
uint8_t save_state = 0;
unsigned long int save_time = 0;
unsigned long int ina_time = 0;
uint8_t send_finished = 0;

void loop() 
{
    if(millis() - ina_time >= ina_interval)
    {
        ina_time = millis();
        readINA219();
    }
    

    if(millis() - prevTime >= 25)
    {
        prevTime += 25;
        receiveData();
    }
    

    if(Global_Time.second_t % 2 == 0)
    {
        digitalWrite(13, HIGH);
    }
    else
    {
        digitalWrite(13, LOW);
    }

    if(!send_finished)
    {
        if(!send_to_user)
        {
            switch(save_state)
            {
                case 0:
                    if(save_ph || save_temp || save_do || save_turb)
                    {
                        writeFiles();
                        save_state++;
                    }
                    break;
                case 1:
                    if(millis() - save_time >= 3)
                    {
                        save_time = millis();
                        save_state = 0;
                        memset(sensor_data_rx, 0, sizeof(sensor_data_rx));
                    }
                    break;
            }
    
        }
        else
        {
            transmitFile();
        }
    }

}

void receiveData()
{
    if(Serial.available())
    {
        static int receive_status;
        static int header_status;
        char temp;

        if(header_status == 0)
        {
            if(Serial.available() >= sizeof(sensor_data_rx) + 3)
            { 
                for(int i = 0; i < sizeof(sensor_data_rx) + 3; i++)
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
            char real_data[sizeof(sensor_data_rx)];
            if(Serial.available() >= sizeof(sensor_data_rx))
            {
                for(int i = 0; i < sizeof(sensor_data_rx); i++)
                {
                    real_data[i] = Serial.read();
                }

                memcpy(&sensor_data_rx, real_data, sizeof(sensor_data_rx));
                
                memcpy(&Global_Time.hour_t, sensor_data_rx, 2);
                memcpy(&Global_Time.minute_t, sensor_data_rx + 2, 1);
                memcpy(&Global_Time.second_t, sensor_data_rx + 3, 1);
                memcpy(&Global_Time.millisecond_t, sensor_data_rx + 4, 2);

                memcpy(&save_ph, sensor_data_rx + 6, 1);
                memcpy(&value_ph, sensor_data_rx + 7, 4);
                memcpy(&interval_ph, sensor_data_rx + 11, 4);

                memcpy(&save_temp, sensor_data_rx + 15, 1);
                memcpy(&value_temp, sensor_data_rx + 16, 4);
                memcpy(&interval_temp, sensor_data_rx + 20, 4);

                memcpy(&save_do, sensor_data_rx + 24, 1);
                memcpy(&value_do, sensor_data_rx + 25, 4);
                memcpy(&interval_do, sensor_data_rx + 29, 4);

                memcpy(&save_turb, sensor_data_rx + 33, 1);
                memcpy(&value_turb, sensor_data_rx + 34, 4);
                memcpy(&interval_turb, sensor_data_rx + 38, 4);

                memcpy(&send_to_user, sensor_data_rx + 42, 1);
                
                header_status = 0;
            }
        }
    }
}

void writeFiles()
{
    String sensor_data_str;
    
    openFile = SD.open("stfu.txt", FILE_WRITE);

    if(openFile)
    {   
        sensor_data_str  = "";
        
        sensor_data_str += String(Global_Time.hour_t); //--- hour
        sensor_data_str += ":";
        sensor_data_str += String(Global_Time.minute_t); //--- minute
        sensor_data_str += ":";
        sensor_data_str += String(Global_Time.second_t); //--- second
        sensor_data_str += ":";
        sensor_data_str += String(Global_Time.millisecond_t); //--- millisecond
        
        sensor_data_str += ";";
        sensor_data_str += String(save_ph);
        sensor_data_str += ";";
        sensor_data_str += String(value_ph);
        sensor_data_str += ";";
        sensor_data_str += String(interval_ph);

        sensor_data_str += ";";
        sensor_data_str += String(save_temp);
        sensor_data_str += ";";
        sensor_data_str += String(value_temp);
        sensor_data_str += ";";
        sensor_data_str += String(interval_temp);
        
        sensor_data_str += ";";
        sensor_data_str += String(save_do);
        sensor_data_str += ";";
        sensor_data_str += String(value_do);
        sensor_data_str += ";";
        sensor_data_str += String(interval_do);
        
        sensor_data_str += ";";
        sensor_data_str += String(save_turb);
        sensor_data_str += ";";
        sensor_data_str += String(value_turb);
        sensor_data_str += ";";
        sensor_data_str += String(interval_turb);
        
        sensor_data_str += ";";
        sensor_data_str += String(current_mA);
        sensor_data_str += ";";
        sensor_data_str += String(loadvoltage);
        
      
        openFile.println(sensor_data_str);
        
        openFile.close();
    }
}

void transmitFile()
{
  openFile = SD.open("STFU.txt");
  if (openFile)
  {
    
    int lineNumber = 0;
    
    // Read until end of file
    //-- openFile.available()
    while (openFile.available())
    {
      lineNumber++;
      int dataIndex = 0;
      String token = "";
      bool lineComplete = false;
      
      // Read a single line
      while (openFile.available() && !lineComplete) 
      {
        char c = openFile.read();
        
        // Check for newline (end of line)
        if (c == '\n')
        {
          lineComplete = true;
        }
        
        // Process data when separator found or at end of line
        if (c == ':' || c == ';' || lineComplete)
        {
          if (token.length() > 0)
          {
            storeData(dataIndex, token);
            dataIndex++;
            token = ""; // Reset token
          }
          if (lineComplete) break;
        } 
        // Skip carriage returns
        else if (c != '\r')
        {
          token += c; // Build current token
        }
      }
      
      // Process line if we got all 18 data points
      if (dataIndex == 18)
      {
        memcpy(sensor_data_tx +  3, &Global_Time.hour_t, 2);
        memcpy(sensor_data_tx +  5, &Global_Time.minute_t, 1);
        memcpy(sensor_data_tx +  6, &Global_Time.second_t, 1);
        memcpy(sensor_data_tx +  7, &Global_Time.millisecond_t, 2);

        memcpy(sensor_data_tx +  9, &save_ph, 1);
        memcpy(sensor_data_tx + 10, &value_ph, 4);
        memcpy(sensor_data_tx + 14, &interval_ph, 4);

        memcpy(sensor_data_tx + 18, &save_temp, 1);
        memcpy(sensor_data_tx + 19, &value_temp, 4);
        memcpy(sensor_data_tx + 23, &interval_temp, 4);

        memcpy(sensor_data_tx + 27, &save_do, 1);
        memcpy(sensor_data_tx + 28, &value_do, 4);
        memcpy(sensor_data_tx + 32, &interval_do, 4);

        memcpy(sensor_data_tx + 36, &save_turb, 1);
        memcpy(sensor_data_tx + 37, &value_turb, 4);
        memcpy(sensor_data_tx + 41, &interval_turb, 4);

        memcpy(sensor_data_tx + 45, &current_mA, 4);
        memcpy(sensor_data_tx + 49, &loadvoltage, 4);

        for(int i = 0; i < sizeof(sensor_data_tx); i++)
        {
            Serial.write(sensor_data_tx[i]);
        }
        delay(5);
            
      } 
      else if (token.length() > 0)
      {
        storeData(dataIndex, token);  // Process last token
      }
    }
    
    openFile.close();
    
    send_finished = 1;
    
    memcpy(sensor_data_tx + 53, &send_finished, 1);

    for(int i = 0; i < 20; i++)
    {
        for(int i = 0; i < sizeof(sensor_data_tx); i++)
        {
            Serial.write(sensor_data_tx[i]);
        }
        delay(5);
    }
  }

}

void storeData(int index, String token) {
  switch(index) {
    case 0: Global_Time.hour_t = token.toInt(); break;
    case 1: Global_Time.minute_t = token.toInt(); break;
    case 2: Global_Time.second_t = token.toInt(); break;
    case 3: Global_Time.millisecond_t = token.toInt(); break;
    case 4: save_ph = token.toInt(); break;
    case 5: value_ph = token.toFloat(); break;
    case 6: interval_ph = token.toFloat(); break;
    case 7: save_temp = token.toInt(); break;
    case 8: value_temp = token.toFloat(); break;
    case 9: interval_temp = token.toFloat(); break;
    case 10: save_do = token.toInt(); break;
    case 11: value_do = token.toFloat(); break;
    case 12: interval_do = token.toFloat(); break;
    case 13: save_turb = token.toInt(); break;
    case 14: value_turb = token.toFloat(); break;
    case 15: interval_turb = token.toFloat(); break;
    case 16: current_mA = token.toFloat(); break;
    case 17: loadvoltage = token.toFloat(); break;
  }
}

void readINA219()
{
//    unsigned long currentMillis = millis();
//
//    if(currentMillis - ina_time >= ina_interval)
//    {
//        ina_time = currentMillis;
        
        shuntvoltage = ina219.getShuntVoltage_mV();
        busvoltage = ina219.getBusVoltage_V();
        current_mA = ina219.getCurrent_mA();
        power_mW = ina219.getPower_mW();
        loadvoltage = busvoltage + (shuntvoltage / 1000);
//    }
}
