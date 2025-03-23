/*
 * Data Logger Program
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include <Adafruit_INA219.h>

SFE_BMP180 pressure;
Adafruit_INA219 ina219;


/*
 * Global Variables
 */
#define chipSelect 4
#define LED_WRITE_PIN 5
#define LED_CONNECT_SD 6

File openFile;

/* Data for sensors
 * 0) Write data status
 * 1) timestamp
 * 2) pressure_water
 * 3) pH
 * 4) temperature_water
 * 5) diss. oxygen
 * 6) turbidity
 * 7) current
 * 8) voltage system
 * 9) voltage motor
 * 10) temperature_inside
 * 11) pressure_inside
 */
#define data_num    12
#define data_size   (data_num * 4) //--- X data * 4 bytes 

char sensor_data_rx[data_size];              //--- "ABC" not stored
char sensor_data_tx[data_size + 3] = "ABC";  //--- "ABC" stored
float sensor_data_f[data_num];
String sensor_data_str = "";

unsigned long int timeNow = 0;
unsigned long int timePrev = 0;

#define ALTITUDE 1655.0 // Altitude of SparkFun's HQ in Boulder, CO. in meters
enum State {
  START_TEMPERATURE,
  GET_TEMPERATURE,
  START_PRESSURE,
  GET_PRESSURE,
  IDLE_STATE
};
State currentState = START_TEMPERATURE;
unsigned long previousMillis = 0;
unsigned long prevTime = 0;
unsigned long waitTime = 0;
double T, P, p0, a;
float temp_BMP180, press_BMP180;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;



void setup()
{
    Serial.begin(115200);
    pinMode(SS, OUTPUT);

    pinMode(LED_CONNECT_SD, OUTPUT);
    pinMode(LED_WRITE_PIN, OUTPUT);

    if (!pressure.begin()) 
    {
        while (1);
    }
    
    if (! ina219.begin())
    {
        while (1);
    }
   
    if (!SD.begin(chipSelect))
    {
        //--- LED OFF
        digitalWrite(LED_CONNECT_SD, LOW);    
        return;
    }
    //--- LED ON
    digitalWrite(LED_CONNECT_SD, HIGH);

    delay(3000);

}

void loop() 
{
    receiveData();

    switch((int)(sensor_data_f[0]))
    {
        case 1:
        {
            writeToFile();
            break;
        }
        case 2:
        {
            transmitFile();
            break;
        }
        default:
        {
            //--- do nothing
            break;
        }
    }

    readBMP180();
    readINA219();
    
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
            if(Serial.available() >= sizeof(sensor_data_rx))
            { 
                for(int i = 0; i < sizeof(sensor_data_rx); i++)
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

                for(int i = 0; i < data_num; i++)
                {
                    memcpy(sensor_data_f + i, sensor_data_rx + (i * sizeof(float)), sizeof(float));
                }
                
                header_status = 0;
            }
        }
    }
}

void writeToFile()
{
    openFile = SD.open("log.txt", FILE_WRITE);
    
    if(openFile)
    {
        digitalWrite(LED_WRITE_PIN, HIGH);
        sensor_data_str = "";
        
        for(int i = 0; i < data_num; i++)
        {
            sensor_data_str += String(sensor_data_f[i]);
            sensor_data_str += ";";
        }
        
        openFile.println(sensor_data_str);
        
        openFile.close();
    }
    
    digitalWrite(LED_WRITE_PIN, LOW);
}

//void writeToFile()
//{
//    openFile = SD.open("log.txt", FILE_WRITE);
//    
//    if(openFile)
//    {
//        digitalWrite(LED_WRITE_PIN, HIGH);
//        sensor_data_str = "";
//        
//
//        sensor_data_str += String(temp_BMP180);
//        sensor_data_str += ";";
//        sensor_data_str += String(press_BMP180);
//        sensor_data_str += ";";
//        sensor_data_str += String(shuntvoltage);
//        sensor_data_str += ";";
//        
//        openFile.println(sensor_data_str);
//        
//        openFile.close();
//    }
//    
//    digitalWrite(LED_WRITE_PIN, LOW);
//}

void transmitFile()
{
    openFile = SD.open("log.txt");

    if(openFile)
    {
        digitalWrite(LED_WRITE_PIN, HIGH);
        while(openFile.available())
        {
            timeNow = millis();

            if(timeNow - timePrev >= 250)
            {
                timePrev = timeNow;

                
            }
            /*
             * Read the file and send the data line by line (periodically)
             */
        }
        openFile.close();
    }
    
    digitalWrite(LED_WRITE_PIN, LOW);
}

void readBMP180()
{
    unsigned long currentMillis = millis();
    char status;
    
    switch (currentState)
    {
        case START_TEMPERATURE:
            status = pressure.startTemperature();
            if (status != 0)
            {
                waitTime = status;
                previousMillis = currentMillis;
                currentState = GET_TEMPERATURE;
            }
            else
            {
                currentState = IDLE_STATE;
            }
            break;
        
        case GET_TEMPERATURE:
            if (currentMillis - previousMillis >= waitTime)
            {
                status = pressure.getTemperature(T);
                if (status != 0)
                {
                    temp_BMP180 = (float)T;
                    currentState = START_PRESSURE;
                }
                else
                {
                    currentState = IDLE_STATE;
                }
            }
            break;
        
        case START_PRESSURE:
            status = pressure.startPressure(3);
            if (status != 0)
            {
                waitTime = status;
                previousMillis = currentMillis;
                currentState = GET_PRESSURE;
            }
            else
            {
                currentState = IDLE_STATE;
            }
            break;
        
        case GET_PRESSURE:
            if (currentMillis - previousMillis >= waitTime)
            {
                status = pressure.getPressure(P, T);
                if (status != 0)
                {
                    press_BMP180 = P;
                    p0 = pressure.sealevel(P, ALTITUDE);
                    a = pressure.altitude(P, p0);
                    
                    currentState = IDLE_STATE;
                }
                else
                {
                    currentState = IDLE_STATE;
                }
            }
            
            break;
        
        case IDLE_STATE:
            if (currentMillis - previousMillis >= 5000)
            {
                currentState = START_TEMPERATURE;
            }
            break;
    }
}


void readINA219()
{
    unsigned long currentMillis = millis();

    if(currentMillis - prevTime >= 2000)
    {
        prevTime = currentMillis;
        
        shuntvoltage = ina219.getShuntVoltage_mV();
        busvoltage = ina219.getBusVoltage_V();
        current_mA = ina219.getCurrent_mA();
        power_mW = ina219.getPower_mW();
        loadvoltage = busvoltage + (shuntvoltage / 1000);
    }
}
