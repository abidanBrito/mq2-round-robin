/*  ------------------------------------------------------------------------
    AUTHOR:        Abidan Brito Clavijo
    FILE:          mq2_round_robin_interrupt.ino
    DATE:          15/01/2021
    STATE:         DONE
    ------------------------------------------------------------------------ */

#include <M5Stack.h>
#include <driver/adc.h>
#include "driver/uart.h"
#include "esp_intr_alloc.h"
//#include <string>

// ------------------------------------------------------------------------------------------
// Compilation directives & Constants
// ------------------------------------------------------------------------------------------
#define SHOW_VALUES 'a'              // Trigger character to print values on the serial monitor
#define MQ_PIN 36                    // Analog input pin (ADC)
#define TX0_PIN 1                    // Serial transmitter pin
#define RX0_PIN 3                    // Serial receiver pin
#define RL_VALUE 5                   // RL module resistance (kOhm)
#define R0 10                        // R0 sensor resistance in clear air (kOhm)
#define READ_SAMPLE_INTERVAL 10      // Sampling time delay
#define READ_SAMPLE_TIMES 200        // Number of samples
#define VCC 5.0                      // Sensor common collector supply voltage
#define ADC_DISCRETE_LEVELS 4095.0   // ADC discrete digital levels (depends on the resolution)
#define BUF_SIZE (1024)              // Buffer size
#define UART_FULL_THRESH_DEFAULT (5) // Number of received bytes to cause the interruption
#define UART_TOUT_THRESH_DEFAULT (0)

// Concentration curve points {X, Y}
// You may change these values according to the Datasheet or the calibration
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
byte count = 0; // 8-bit unsigned integer

// Timer0 declaration
hw_timer_t *timer0 = NULL;

// ISR (interruption) handle
static intr_handle_t handle_console;

// Reception buffer
volatile uint8_t rxbuf[BUF_SIZE];

// Flag to trigger the subroutine associated to the ISR
volatile int8_t Flag_uart_int = 0;

// Flags
volatile int Flag_ISR_Timer0 = 0;
volatile int Flag_UART = 0;
volatile int Flag_LCD = 0;

int raw_adc;
float value_adc;

float measurements[200] = {};
float minRaw = 10000;
float maxRaw = 0;
float meanRaw = 0;
float mean = 0;
float meanVoltage = 0;
float meanVoltagePrint = 0;

void IRAM_ATTR ISR_Timer0()
{
   if (count == READ_SAMPLE_TIMES)
   {
      count = 0;
   }

   circularBuffer[count] = analogRead(MQ_PIN);
   count++;

   if (count == 99 || count == 199)
   {
      Flag_LCD = 1;
   }

   Flag_ISR_Timer0 = 1;
}

// UART interruption subroutine
static void IRAM_ATTR ISR_UART(void *arg)
{
   uint8_t rx_fifo_len = 10; // State
   uint8_t i = 0;

   // Read the number of bytes from the UART buffer
   rx_fifo_len = UART0.status.rxfifo_cnt;

   // Read bytes from the buffer
   while (rx_fifo_len)
   {
      rxbuf[i++] = UART0.fifo.rw_byte;
      rx_fifo_len--;
      Flag_uart_int++;
   }

   // Clear status bit from the UART interruption
   uart_clear_intr_status(UART_NUM_0, UART_RXFIFO_FULL_INT_CLR);
}

// ------------------------------------------------------------------------------------------
// Setup
// ------------------------------------------------------------------------------------------
void setup()
{
   Serial.begin(115200);        // Data rate (baud) for serial data transmission
   pinMode(MQ_PIN, INPUT);

   // Find out the concentration and print it to the serial monitor
   Serial.println("Calibrating...\n");
   float rs_med = readMQ(MQ_PIN); // Mean Rs value
   float concentration = getConcentration(rs_med / R0);
   Serial.print("Concentration: ");
   Serial.println(concentration);

   // ADC configuration
   adcAttachPin(MQ_PIN);
   adcStart(MQ_PIN);
   analogSetWidth(12);
   adc1_config_width(ADC_WIDTH_BIT_12);
   adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_2_5db);
   
   M5.begin(true, false, true); // Enable LCD & Serial, disable SD

   // Timer0 configuration
   // Timer0 (TIMG0_T0), Periodo TB_clk = 12.5 ns * TIMGn_Tx_CLK_PRESCALE = 12.5 ns * 80
   // -> 1000 ns = 1 us, countUp
   timer0 = timerBegin(0, 80, true);

   // Interruption configuration
   timerAttachInterrupt(timer0, &ISR_Timer0, true); // edge (not level) triggered

   // Write alarm threshold value (con autorecarga)
   // Alarm period = 1000000 * 1 us = 1 s
   timerAlarmWrite(timer0, 10000, true);

   // Enable the alarm
   timerAlarmEnable(timer0);

   // Disable the UART0, in order to configure it using ESP-IDF functions
   Serial.end();

   // UART configuration / initialization
   uart_config_t uart_config = {
       .baud_rate = 115200,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .rx_flow_ctrl_thresh = 120,
       .use_ref_tick = false};

   // uart_param_config(uart_port_t uart_num, const uart_config_t *uart_config);
   uart_param_config(UART_NUM_0, &uart_config);

   // uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num);
   uart_set_pin(UART_NUM_0, TX0_PIN, RX0_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

   // uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size,
   uart_driver_install(UART_NUM_0, BUF_SIZE, 0, 0, NULL, 0);

   // Free UART ISR handle
   uart_isr_free(UART_NUM_0);

   // Register UART ISR
   uart_isr_register(UART_NUM_0, ISR_UART, NULL, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1, &handle_console);

   // UART interruption configuration
   uart_intr_config_t uart0_intr;
   uart0_intr.intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M;
   uart0_intr.rxfifo_full_thresh = UART_FULL_THRESH_DEFAULT;
   uart0_intr.rx_timeout_thresh = UART_TOUT_THRESH_DEFAULT;

   uart_intr_config(UART_NUM_0, &uart0_intr);

   // Enable interruptions mask
   uart_enable_intr_mask(UART_NUM_0, UART_RXFIFO_FULL_INT_ENA);
}

// ------------------------------------------------------------------------------------------
// Loop
// ------------------------------------------------------------------------------------------
uint8_t readChar = 0;
void loop()
{
   // ISR 1 (actions)
   if (Flag_ISR_Timer0 == 1)
   {
      float value_adc = circularBuffer[count] * (VCC / ADC_DISCRETE_LEVELS);

      maxRaw = 0;
      minRaw = 10000;
      mean = 0;
      meanVoltage = 0;
      meanRaw = 0;
      meanVoltagePrint = 0;

      // 200 samples
      for (int i = 0; i < READ_SAMPLE_TIMES; i++)
      {
         if (circularBuffer[i] > maxRaw)
         {
            maxRaw = circularBuffer[i];
         }

         if (circularBuffer[i] < minRaw)
         {
            minRaw = circularBuffer[i];
         }

         meanRaw += circularBuffer[i];
         meanVoltagePrint += (circularBuffer[i] * (VCC / ADC_DISCRETE_LEVELS));
      }

      // 100 samples (LCD)
      for (int i = 0; i < 100; i++)
      {
         mean += circularBuffer[i];
         meanVoltage += (circularBuffer[i] * (VCC / ADC_DISCRETE_LEVELS));
      }

      if (Flag_LCD == 1)
      {
         M5.Lcd.clear();
         M5.Lcd.setCursor(0, 0);
         M5.Lcd.setTextSize(2);

         // Raw values
         M5.Lcd.print("Raw min:  ");
         M5.Lcd.println(minRaw); // Print floating point number
         M5.Lcd.print("Raw max:  ");
         M5.Lcd.println(maxRaw); // Print as integer value in binary
         M5.Lcd.print("Raw mean: ");
         M5.Lcd.println(mean / 100); // Print as integer number in Hexadecimal

         // Voltage values
         M5.Lcd.print("Voltage min:  ");
         M5.Lcd.println(minRaw * (VCC / ADC_DISCRETE_LEVELS)); // Print floating point number
         M5.Lcd.print("Voltage max:  ");
         M5.Lcd.println(maxRaw * (VCC / ADC_DISCRETE_LEVELS)); // Print as integer value in binary
         M5.Lcd.print("Voltage mean:  ");
         M5.Lcd.println(meanVoltage / 100); // Print as integer number in Hexadecimal

         Flag_LCD = 0;
      }

      Flag_ISR_Timer0 = 0;
   }

   if (Flag_uart_int > 0)
   {
      for (uint8_t i = Flag_uart_int; i; i--)
      {
         if (rxbuf[i - 1] == 'a')
         {
            readChar = 'a';
         }
      }

      if (readChar == 'a')
      {
         readChar = 0;
         Flag_UART = 1;
      }

      if (Flag_UART == 1)
      {
         char buffer[1000];

         // Raw values
         snprintf(buffer, sizeof(buffer), "Raw min: %f", minRaw);
         puts(buffer);
         snprintf(buffer, sizeof(buffer), "Raw max: %f", maxRaw);
         puts(buffer);
         snprintf(buffer, sizeof(buffer), "Raw mean: %f", meanRaw / READ_SAMPLE_TIMES);
         puts(buffer);

         // Voltage values
         snprintf(buffer, sizeof(buffer), "Voltage min: %f V", minRaw * (VCC / ADC_DISCRETE_LEVELS));
         puts(buffer);
         snprintf(buffer, sizeof(buffer), "Voltage max: %f V", maxRaw * (VCC / ADC_DISCRETE_LEVELS));
         puts(buffer);
         snprintf(buffer, sizeof(buffer), "Voltage mean: %f V", meanVoltagePrint / READ_SAMPLE_TIMES);
         puts(buffer);

         Flag_UART = 0;
      }

      Flag_uart_int = 0;
   }
}

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

float getValueAdc()
{
   int raw_adc = analogRead(MQ_PIN);
   float value_adc = raw_adc * (VCC / ADC_DISCRETE_LEVELS);

   return value_adc;
}