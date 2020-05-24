// This #include statement was automatically added by the Particle IDE.
#include "roomba.h"

/*-----------------------------------------------------------------------------
 ** Global Defines/Typedefs/Enums/Macros.
 **
 **  pin3Rxd -> White wire -> Photon TX pin
 **  pin4Txd -> Green wire -> Photon RX pin
 **  pin5Brc -> Black Wire -> Photon pin D5
 **---------------------------------------------------------------------------*/
din7Connector roombaDin = {.pin1Vpwr = 0, .pin2Vpwr = 0, .pin3Rxd = 6, .pin4Txd = 7, .pin5Brc = D5, .pin6Gnd = 0, .pin7Gnd = 0};
roombaObj artuRoomba = {.roombaState = ASLEEP, .roombaUpdateFunc = roombaCmdFromSleep};
sensorPacketList sensors = {.chargingState = {21, 1, false}, .voltage = {22, 2, false}, .current = {23, 2, true}, .temperature = {24, 1, true}, .batteryCharge = {25, 2, false}, .batteryCapacity = {26, 2, false}};

unsigned long int roombaIOtimeout = 240000; // 4 min (actually less? double check)
unsigned long roombaTimeMark = 0;
int ledPin = D7;
int ledState = 0;
int roombaCmd = 0;
boolean async = false;
String inSerial = "";
const size_t READ_BUF_SIZE = 1024;
size_t readBufOffset = 0;
char readBuf[READ_BUF_SIZE];
boolean readPromise = false;
int readOpCode = 0;
int readTries = 0;
int activeTimer = 0;

int period = 1000;
unsigned long time_now = 0;

// Arduino test for the sensor bytes

int sensorbytes[20]; //array to store encoder counts
int angle;
const float pi = 3.1415926;
#define left_encoder (sensorbytes[0])
#define right_encoder (sensorbytes[1])
int packetID = 7;

//---------------------Setup()---------------------------------
void setup()
{

    Serial1.begin(115200, SERIAL_FLOW_CONTROL_NONE); // for Roomba communication via RX/TX pins

    //     pinMode(roombaDin.pin5Brc, OUTPUT);         // Roomba's BRC pin. Used to wake it up
    pinMode(D5, OUTPUT);

    pinMode(ledPin, OUTPUT);   // use built in led for feedback
    roombaTimeMark = millis(); // start timer

    // set up API connection

    wakeUp(roombaDin);

    Particle.function("webCmd", receiveWebCmd);

    RGB.control(true);
    cmdRoomba(RESET);

    // Serial.println("Setup complete");
    Particle.publish("DEBUG", "Setup has completed");
}

//---------------------Spark web cmd()---------------------------------
int receiveWebCmd(String command)
{

    roombaCmd = command.toInt();

    //Particle.publish("DEBUG", "Command from Web: " + command);

    // Try a wake up
    //digitalWrite(D5, LOW); // line must start high
    //delay(100);
    //digitalWrite(D5, HIGH); // 500ms LOW to wake up
    //delay(2000);

    return 1;
}
//https://build.particle.io/build/5eae4b76c9a6d3000792ed79#flash}

/*---------------------Main Loop()---------------------------------
 - Receive cmds from Serial or API
 - Perform cmd and update state
 - check timers and read Roomba response
 ----------------------------------------------------------------*/
void loop()
{
    /*---------------------Receive cmds from Serial or API---------------------------------*/

    // Commanding from serial monitor. Ascii to decimal
    // examples:  "*" -> 128;     "1" -> 135;       "9" -> 143,      "?" -> 149

    // wakeUp(roombaDin); // lets see this works - keep pulsing the pin

    // receiveWebCmd happens asynchronously
    /*---------------------Perform cmd and update state---------------------------------*/

    if (activeTimer > millis())
    {
        // Wrapr round time
        activeTimer = 0;
    }
    if ((millis() - activeTimer) > 60000)
    {
        activeTimer = millis();
        Particle.publish("KEEPALIVE", String(activeTimer));
        cmdRoomba(START_OI);
        cmdRoomba(START_OI);
    }

    if (readPromise && readOpCode == 0)
    {
        // We have an out of sync postion - ready the bufffer and send it to diagnostics
        Particle.publish("BUFFER", readBuf);
        readPromise = false;
    }

    if (readOpCode > 0 && readPromise)
    {

        //Particle.publish("DEBUG", "Read Promise");
        //Particle.publish("DEBUG_VAL",String(readOpCode));
        unsigned int byte1 = 0;
        unsigned int byte2 = 0;

        if (readOpCode == SENSOR_DIRT)
        {
            byte1 = (int)readBuf[0];
            Particle.publish("READ", "DIRT Sensor Value");
            Particle.publish("VALUE", String(byte1));
        }

        if (readOpCode == 21)
        {
            byte1 = (int)readBuf[0];
            Particle.publish("READ", "Charging State");
            Particle.publish("VALUE", String(byte1));
        }

        if (readOpCode == 26)
        {
            byte1 = (int)readBuf[0] + (int)readBuf[1] * 256;
            Particle.publish("READ", "Capacity");
            Particle.publish("VALUE", String(byte1));
        }
        if (readOpCode == 25)
        {
            byte1 = (int)readBuf[0] + (int)readBuf[1] * 256;
            Particle.publish("READ", "Charge Remaining");
            Particle.publish("VALUE", String(byte1));
        }

        // Main motor current draw
        if (readOpCode == 57)
        {
            byte1 = (int)readBuf[0] + (int)readBuf[1] * 256;
            Particle.publish("READ", "Motor Current");
            Particle.publish("VALUE", String(byte1));
        }

        if (readOpCode == 35)
        {
            byte1 = (unsigned int)readBuf[0];
            Particle.publish("READ", "OI Mode");
            Particle.publish("VALUE", String(byte1));
        }

        readOpCode = 0;
        readPromise = false;
    }

    if (roombaCmd != 0 || !async)
    {
        //Serial.print("--Cmd: ");

        if ((roombaCmd >= 8) && (roombaCmd <= 58))
        {

            cmdRoomba(START_OI);
            Particle.publish("READOP", "Roomba READ - 142 OpCode");
            cmdRoombaZero(142);
            readOpCode = roombaCmd;
            cmdRoomba(roombaCmd);

            RGB.color(255, 0, 0);
        }
        else
        {
            // Now we can see this as a start up

            if (roombaCmd == CLEAN)
            {

                //wakeUp(roombaDin);
                //delay(2000);

                if (!async)
                {
                    cmdRoomba(START_OI);
                    cmdRoomba(START_OI);

                    cmdRoomba(CLEAN);

                    // Perform a reset - but dont execute the command until we have passwed X time

                    Particle.publish("OP", "Roomba Async");
                    async = true;
                    time_now = millis();
                    return;
                }
                else
                {

                    int now = millis();
                    if (now > time_now + 5000)
                    {
                        //wakeUp(roombaDin);

                        async = false;
                        //cmdRoomba(START_OI);
                        //delay(1000);
                        cmdRoomba(CLEAN);
                        Particle.publish("OP", "CLEAN");
                    }
                    else
                    {
                        return;
                    }
                }
            }
            else if (roombaCmd == SEEK_DOCK)
            {

                if (!async)
                {

                    cmdRoomba(START_OI);
                    cmdRoomba(START_OI);
                    cmdRoomba(SEEK_DOCK);

                    // Perform a reset - but dont execute the command until we have passwed X time
                    //cmdRoomba(7);

                    Particle.publish("OP", "Roomba Async");
                    async = true;
                    time_now = millis();
                    return;
                }
                else
                {

                    int now = millis();
                    if (now > time_now + 5000)
                    {
                        async = false;
                        Particle.publish("OP", "DOCK");
                        cmdRoomba(START_OI);
                        cmdRoomba(SEEK_DOCK);
                    }
                    else
                    {
                        return;
                    }
                }
            }

            else if (roombaCmd == POWER_DOWN)
            {

                Particle.publish("OP", "STOP");
                //wakeUp(roombaDin);

                cmdRoomba(START_OI);
                delay(1000);
                cmdRoomba(POWER_DOWN);
            }

            else if (roombaCmd == START_OI)
            {

                Particle.publish("OP", "START OI");

                cmdRoomba(START_OI);
            }

            else if (roombaCmd == RESET)
            {

                Particle.publish("OP", "RESET");
                //wakeUp(roombaDin);
                wakeUp(roombaDin);
                delay(1000);
                cmdRoomba(START_OI);
                delay(1000);
                cmdRoomba(RESET);
            }

            else if (roombaCmd == STOP_OI)
            {

                Particle.publish("OP", "STOP OI");
                //wakeUp(roombaDin);

                cmdRoomba(STOP_OI);
            }
            else if (roombaCmd > 0)
            {
                Particle.publish("OPVAL", String(roombaCmd));
                //wakeUp(roombaDin);

                cmdRoomba(roombaCmd);
            }
        }

        roombaCmd = 0; // reset received cmd
        async = false;

    } // End if(roombaCmd)

    // sample serial received msgs that are not sensor packets.

    int bytesRead = 0;
    while (Serial1.available())
    {

        if (bytesRead == 0)
        {
            memset(&readBuf[0], 0, READ_BUF_SIZE);
        }
        char c = Serial1.read();

        readBuf[bytesRead++] = c;
        readPromise = true;
    }
    if (bytesRead)
    {

        Particle.publish("PROMISE", String(bytesRead));

        RGB.color(0, 0, 0);
        bytesRead = 0;
    }
}
