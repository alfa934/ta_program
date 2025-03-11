/*
 * Data Logger Program
 */

#include <SPI.h>
#include <SD.h>

/*
 * Global Variables
 */
#define chipSelect 4
#define LED_WRITE_PIN 13
#define LED_CONNECT_SD 12

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
float sensor_data_f[data_num]
String sensor_data_str = "";

unsigned long int timeNow = 0;
unsigned long int timePrev = 0;

void setup()
{
    Serial.begin(115200);
    pinMode(SS, OUTPUT);
   
    if (!SD.begin(chipSelect))
    {
        //--- LED OFF    
        return;
    }
    //--- LED ON

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
        //--- LED ON
        sensor_data_str = "";
        
        for(int i = 0; i < data_num; i++)
        {
            sensor_data_str += String(sensor_data_f[i]);
            sensor_data_str += ";";
        }
        
        openFile.println(sensor_data_str);
        
        openFile.close();
    }
    else
    {
        //--- LED OFF
    }
}

void transmitFile()
{
    openFile = SD.open("log.txt");

    if(openFile)
    {
        //--- LED ON
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
    else
    {
        //--- LED OFF
    }
}
