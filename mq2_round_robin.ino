/*  ------------------------------------------------------------------------
    AUTHOR:        Abidan Brito Clavijo
    FILE:          mq2_round_robin.ino
    DATE:          15/01/2021
    STATE:         DONE
    ------------------------------------------------------------------------ */

#include <M5Stack.h>

// ------------------------------------------------------------------------------------------
// Compilation directives & Constants
// ------------------------------------------------------------------------------------------
#define SHOW_VALUES 'a'            // Trigger character to print values on the serial monitor
#define MQ_PIN 36                  // Analog input pin (ADC)
#define RL_VALUE 5                 // RL module resistance (kOhm)
#define R0 10                      // R0 sensor resistance in clear air (kOhm)
#define READ_SAMPLE_INTERVAL 10    // Sampling time delay
#define READ_SAMPLE_TIMES 200      // Number of samples
#define VCC 5.0                    // Sensor common collector supply voltage
#define ADC_DISCRETE_LEVELS 4095.0 // ADC discrete digital levels (depends on the resolution)
#define MAX_R_VALUE 0              // Raw maximum value threshold
#define MIN_R_VALUE 10000          // Raw minimum value threshold
#define MID_R_VALUE 0              // Raw mean value threshold

// Concentration curve points {X, Y}
const float p0[] = {log10(200), log10(1.7)};
const float p1[] = {log10(10000), log10(0.28)};

// Slope and abscissa coordinate
const float scope = (p1[1] - p0[1]) / (p1[0] - p0[0]);
const float coord = p0[1] - p0[0] * scope;

// ------------------------------------------------------------------------------------------
// Circular buffer (to hold the samples)
// ------------------------------------------------------------------------------------------
const unsigned int circularBufferLength = READ_SAMPLE_TIMES;
int circularBuffer[circularBufferLength];
int circularBufferIndex = 0;
byte count = 0; // 8-bit unsigned integer

// ------------------------------------------------------------------------------------------
// Structures (to hold the values)
// ------------------------------------------------------------------------------------------
struct RawValues
{
   float min, max, mean;
};

struct VoltageValues
{
   float min, max, mean;
};

// ------------------------------------------------------------------------------------------
// Setup & Loop
// ------------------------------------------------------------------------------------------
void setup()
{
   M5.begin(true, false, true); // Enable LCD & Serial, disable SD
   Serial.begin(115200);        // Data rate (baud) for serial data transmission
   pinMode(MQ_PIN, INPUT);

   // Find out the concentration and print it to the serial monitor
   Serial.println("Calibrating...\n");
   float rs_med = readMQ(MQ_PIN); // Mean Rs value
   float concentration = getConcentration(rs_med / R0);
   Serial.print("Concentration: ");
   Serial.println(concentration);
}

void loop()
{
   if (circularBufferIndex == READ_SAMPLE_TIMES)
   {
      circularBufferIndex = 0; // Reset circular buffer index
   }

   // Perform a new ADC reading
   circularBuffer[circularBufferIndex] = analogRead(MQ_PIN);

   // Update counters
   circularBufferIndex++;
   count++;

   // Retrieve 200 samples
   int data[READ_SAMPLE_TIMES];
   int M = sizeof(data) / sizeof(int);
   getFromBuffer(data, M);

   // Structures to hold the values to be printed
   struct RawValues raw;
   struct VoltageValues volt;
   raw.max = MAX_R_VALUE;
   raw.min = MIN_R_VALUE;
   raw.mean = MID_R_VALUE;

   for (int i = 0; i < READ_SAMPLE_TIMES; i++)
   {
      if (data[i] > raw.max)
      {
         raw.max = data[i]; // Maximum raw value
      }

      if (data[i] < raw.min)
      {
         raw.min = data[i]; // Minimum raw value
      }

      raw.mean += data[i]; // Mean raw value
   }

   // Voltage values
   volt.min = raw.min * (VCC / ADC_DISCRETE_LEVELS);
   volt.max = raw.max * (VCC / ADC_DISCRETE_LEVELS);
   volt.mean = (raw.mean * (VCC / ADC_DISCRETE_LEVELS)) / READ_SAMPLE_TIMES;

   // Print to the LCD screen every 100 new samples
   if (count == 100)
   {
      printValuesLcd(raw.min, raw.max, raw.mean, volt.min, volt.max, volt.mean);
      count = 0;
   }

   // Print to the serial monitor whenever 'a' gets entered
   if (Serial.available() > 0)
   {
      char readChar = Serial.read();
      if (readChar == SHOW_VALUES)
      {
         printValuesSerial(raw.min, raw.max, raw.mean, volt.min, volt.max, volt.mean);
      }
   }

   delay(READ_SAMPLE_INTERVAL);
}
// ------------------------------------------------------------------------------------------

// Returns the mean resistance of N samples
float readMQ(int mq_pin)
{
   float rs = 0;
   for (int i = 0; i < READ_SAMPLE_TIMES; i++)
   {
      rs += getMQResistance(analogRead(mq_pin));
      delay(READ_SAMPLE_INTERVAL);
   }

   return rs / READ_SAMPLE_TIMES;
}

// Returns the resistance from the analog reading
float getMQResistance(int raw_adc)
{
   return (((float)RL_VALUE / 1000.0 * (ADC_DISCRETE_LEVELS - raw_adc) / raw_adc));
}

// Returns concentration 10^(coord + scope * log (rs / r0)
float getConcentration(float rs_ro_ratio)
{
   return pow(10, coord + scope * log(rs_ro_ratio));
}

// Retrieves data from the circular buffer through a pointer
void getFromBuffer(int *out, int outLength)
{
   int readIndex = (circularBufferIndex - outLength + circularBufferLength) % circularBufferLength;
   for (int i = 0; i < outLength; i++)
   {
      if (readIndex >= circularBufferLength)
      {
         readIndex = 0;
      }
      out[i] = circularBuffer[readIndex];
      readIndex++;
   }
}

// Prints out the values to the LCD screen of the M5Stack
void printValuesLcd(float minRaw, float maxRaw, float meanRaw,
                    float minVolt, float maxVolt, float meanVolt)
{
   M5.Lcd.clear();
   M5.Lcd.setTextSize(2);
   M5.Lcd.setCursor(0, 0);

   M5.Lcd.print("\nRaw min: ");
   M5.Lcd.println(minRaw);
   M5.Lcd.print("\nRaw max: ");
   M5.Lcd.println(maxRaw);
   M5.Lcd.print("\nRaw mean: ");
   M5.Lcd.println(meanRaw / READ_SAMPLE_TIMES);

   M5.Lcd.print("\nVoltage min: ");
   M5.Lcd.println(minVolt);
   M5.Lcd.print("\nVoltage max: ");
   M5.Lcd.println(maxVolt);
   M5.Lcd.print("\nVoltage mean: ");
   M5.Lcd.println(meanVolt);
}

// Prints out the values to the Serial Monitor
void printValuesSerial(float minRaw, float maxRaw, float meanRaw,
                       float minVolt, float maxVolt, float meanVolt)
{
   Serial.print("\nRaw min: ");
   Serial.println(minRaw);
   Serial.print("\nRaw max: ");
   Serial.println(maxRaw);
   Serial.print("\nRaw mean: ");
   Serial.println(meanRaw / READ_SAMPLE_TIMES);

   Serial.print("\nVoltage min: ");
   Serial.println(minVolt);
   Serial.print("\nVoltage max: ");
   Serial.println(maxVolt);
   Serial.print("\nVoltage mean: ");
   Serial.println(meanVolt);
}
