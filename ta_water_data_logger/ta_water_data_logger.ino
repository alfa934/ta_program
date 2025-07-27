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

char sensor_data_rx[50];

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

    // Wait for first valid reading before creating file
    while(!(current_mA && loadvoltage))
    {
        readINA219();
        delay(100);
    }
    
    // Write an initial line to the file
    writeFiles();
}

uint8_t save_state = 0;
unsigned long int save_time = 0;
unsigned long int ina_time = 0;
uint8_t send_finished = 0;

void loop() 
{
    // Always check for new data from the ESP32
    receiveData();

    // Handle other tasks like reading the INA219 sensor and blinking the LED
    if(millis() - ina_time >= ina_interval)
    {
        ina_time = millis();
        readINA219();
    }
    
    if(Global_Time.second_t % 2 == 0)
    {
        digitalWrite(13, HIGH);
    }
    else
    {
        digitalWrite(13, LOW);
    }

    // Corrected Logic Loop
    // ACTION 1: If the command is to fetch data, do it and then stop.
    if(send_to_user && !send_finished)
    {
        transmitFile();
        send_finished = 1; // Mark as finished so it only runs once
    }
    // ACTION 2: If we are not fetching data, then log any new sensor readings.
    else if (!send_to_user)
    {
        // State machine for writing to the file
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
                    if(receive_status == 0 && temp == 'A')      receive_status++;
                    else if(receive_status == 1 && temp == 'B') receive_status++;
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

// This function is memory-safe.
void writeFiles()
{
    openFile = SD.open("stfu.txt", FILE_WRITE);

    if(openFile)
    {   
        openFile.print(Global_Time.hour_t);
        openFile.print(":");
        openFile.print(Global_Time.minute_t);
        openFile.print(":");
        openFile.print(Global_Time.second_t);
        openFile.print(":");
        openFile.print(Global_Time.millisecond_t);
        openFile.print(";");
        openFile.print(save_ph);
        openFile.print(";");
        openFile.print(value_ph);
        openFile.print(";");
        openFile.print(interval_ph);
        openFile.print(";");
        openFile.print(save_temp);
        openFile.print(";");
        openFile.print(value_temp);
        openFile.print(";");
        openFile.print(interval_temp);
        openFile.print(";");
        openFile.print(save_do);
        openFile.print(";");
        openFile.print(value_do);
        openFile.print(";");
        openFile.print(interval_do);
        openFile.print(";");
        openFile.print(save_turb);
        openFile.print(";");
        openFile.print(value_turb);
        openFile.print(";");
        openFile.print(interval_turb);
        openFile.print(";");
        openFile.print(current_mA);
        openFile.print(";");
        openFile.println(loadvoltage);
        
        openFile.close();
    }
}

// --- FINAL CORRECTED FUNCTION ---
// This version is completely memory-safe and will not crash the Nano.
// It sends the raw text data from the SD card, which the ESP32 is expecting.
void transmitFile()
{
  openFile = SD.open("STFU.txt");
  if (openFile)
  {
    // This protocol sends the raw text data for maximum reliability
    while (openFile.available())
    {
      // Read a small, safe chunk of the file into a buffer
      char buffer[64];
      int bytesRead = openFile.read(buffer, sizeof(buffer));
      if (bytesRead > 0) {
        // Write that chunk directly to the Serial port
        Serial.write((uint8_t*)buffer, bytesRead);
      }
    }
    openFile.close();

    // After sending the whole file, send a single, clear "finished" message.
    // The last token '1' will signal the GUI via the ESP32 that the process is done.
    Serial.print("0:0:0:0;0;0.00;0.00;0;0.00;0.00;0;0.00;0.00;0;0.00;0.00;0.00;0.00;1\n");
  }
}

// This function is no longer used by the new, safer transmitFile logic.
void storeData(int index, String token) {
    // This function can be left here or deleted.
}

void readINA219()
{
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
}
