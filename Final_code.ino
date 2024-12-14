#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>
#include <string.h>
#include <DHT.h>

#define RECEIVE_BUFFER_EMPTY 0x80
#define TRANSMIT_BUFFER_EMPTY 0x20

#define LCD_MAX_LENGTH 16
#define LCD_PIN_RS 12
#define LCD_PIN_ENABLE 11
#define LCD_PIN_D4 6
#define LCD_PIN_D5 5
#define LCD_PIN_D6 4
#define LCD_PIN_D7 3

#define DHT_SENSOR_PIN 7
#define DHT_SENSOR_TYPE DHT11

#define STEPPER_REVOLUTION_STEPS 2038

#define FAN_CONTROL_MASK 0x10

#define GREEN_LED_MASK 0x80
#define YELLOW_LED_MASK 0x20
#define RED_LED_MASK 0x08
#define BLUE_LED_MASK 0x02

#define START_BUTTON_MASK 0x08
#define RESET_BUTTON_MASK 0x04
#define CONTROL_BUTTON_MASK 0x02

#define PIN_CHANGE_INTERRUPT_MASK 0x0C  // Use to enable PC-Interrupts on PCINT2 and PCINT3 (ST and RES buttons)

// Registers for Digital Pins 50 - 53, 10 - 13 (Buttons)
volatile unsigned char *PORT_BUTTONS = (unsigned char *) 0x25;
volatile unsigned char *DDR_BUTTONS = (unsigned char *) 0x24;
volatile unsigned char *PIN_BUTTONS = (unsigned char *) 0x23;

// Registers for Digital Pins 30 - 37         (LEDs)
volatile unsigned char *PORT_LEDS = (unsigned char *) 0x28;
volatile unsigned char *DDR_LEDS = (unsigned char *) 0x27;
volatile unsigned char *PIN_LEDS = (unsigned char *) 0x26;

// Registers for ADC                          (Level Sensor)
volatile unsigned char *ADC_MUX_REGISTER = (unsigned char *) 0x7C;
volatile unsigned char *ADC_CONTROL_STATUS_B = (unsigned char *) 0x7B;
volatile unsigned char *ADC_CONTROL_STATUS_A = (unsigned char *) 0x7A;
volatile unsigned int *ADC_DATA_REGISTER = (unsigned int *) 0x78;

// Registers for UART0                        (Serial Printing)
volatile unsigned char *UART_CONTROL_STATUS_A = (unsigned char *) 0x00C0;
volatile unsigned char *UART_CONTROL_STATUS_B = (unsigned char *) 0x00C1;
volatile unsigned char *UART_CONTROL_STATUS_C = (unsigned char *) 0x00C2;
volatile unsigned char *UART_BAUD_RATE_REGISTER = (unsigned char *) 0x00C4;
volatile unsigned char *UART_DATA_REGISTER = (unsigned char *) 0x00C6;

// Registers for Pin Change Interrupts
volatile unsigned char *PC_INTERRUPT_CONTROL_REGISTER = (unsigned char *) 0x68;
volatile unsigned char *PC_INTERRUPT_FLAG_REGISTER = (unsigned char *) 0x3B;
volatile unsigned char *PC_INTERRUPT_MASK_REGISTER = (unsigned char *) 0x6B;

typedef enum SYSTEM_STATE {SYSTEM_DISABLED, SYSTEM_IDLE, SYSTEM_ERROR, SYSTEM_RUNNING} SYSTEM_STATE;

LiquidCrystal lcd(LCD_PIN_RS, LCD_PIN_ENABLE, LCD_PIN_D4, LCD_PIN_D5, LCD_PIN_D6, LCD_PIN_D7);
Stepper stepper_motor = Stepper(STEPPER_REVOLUTION_STEPS, 28, 26, 24, 22);
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
RTC_DS3231 real_time_clock;

SYSTEM_STATE current_state = SYSTEM_DISABLED;
SYSTEM_STATE previous_state;

char lcd_buffer[LCD_MAX_LENGTH], error_message[LCD_MAX_LENGTH];
char state_labels[4][16] = {"(DISABLED)", "IDLE", "ERROR", "RUNNING"};
unsigned char led_state_masks[4] = {YELLOW_LED_MASK, GREEN_LED_MASK, RED_LED_MASK, BLUE_LED_MASK};

const unsigned int temperature_threshold = 30;
const unsigned int water_level_threshold = 400;
const unsigned int max_stepper_steps = 200;
unsigned int water_level = 0;
unsigned int update_timer = 0;

void setup() {
  lcd.begin(16, 2);
  dht_sensor.begin();
  real_time_clock.begin();
  real_time_clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
  lcd.clear();
  stepper_motor.setSpeed(2);

  initialize_io();
  initialize_adc();
  initialize_uart(9600);
}

void loop() {
  // Get the current time
  DateTime current_time = real_time_clock.now();

  // Read the ADC value for the water level
  water_level = read_adc(0);

  // Water detection logic (skip in DISABLED or ERROR)
  if (current_state != SYSTEM_DISABLED && current_state != SYSTEM_ERROR) {
    if (water_level <= 150) { // Low water
      snprintf(error_message, LCD_MAX_LENGTH, "Low water!");
      current_state = SYSTEM_ERROR; // Transition to ERROR
    } else if (current_state != SYSTEM_RUNNING) {
      current_state = SYSTEM_IDLE;  // Transition to IDLE if water is detected and not in RUNNING
    }
  }

  // Handle state-specific logic
  switch (current_state) {
    case SYSTEM_DISABLED:
      *PORT_BUTTONS &= ~FAN_CONTROL_MASK; // Turn off the fan
      lcd.setCursor(0, 0);
      lcd.print("System Disabled"); // Display disabled message
      lcd.setCursor(0, 1);
      lcd.print("                ");  // Clear second row
      break;

    case SYSTEM_IDLE:
      *PORT_BUTTONS &= ~FAN_CONTROL_MASK; // Turn off the fan
      lcd.setCursor(0, 0);
      refresh_temperature_humidity(lcd_buffer); // Refresh temperature and humidity
      lcd.print(lcd_buffer);  // Display temperature and humidity
      lcd.setCursor(0, 1);
      lcd.print(state_labels[current_state]); // Display current state
      if ((int)dht_sensor.readTemperature(true) >= temperature_threshold) {
        current_state = SYSTEM_RUNNING; // Transition to RUNNING if temperature exceeds threshold
      }
      break;

    case SYSTEM_ERROR:
      *PORT_BUTTONS &= ~FAN_CONTROL_MASK; // Ensure fan is off in ERROR
      lcd.setCursor(0, 0);
      lcd.print(error_message); // Display error message
      lcd.setCursor(0, 1);
      lcd.print(state_labels[current_state]); // Display current state
      break;

    case SYSTEM_RUNNING:
      lcd.setCursor(0, 0);
      refresh_temperature_humidity(lcd_buffer); // Refresh temperature and humidity
      lcd.print(lcd_buffer);  // Display temperature and humidity
      lcd.setCursor(0, 1);
      lcd.print(state_labels[current_state]); // Display current state
      *PORT_BUTTONS |= FAN_CONTROL_MASK;  // Turn on the fan
      if (water_level <= 150) { // Low water detected in RUNNING
        snprintf(error_message, LCD_MAX_LENGTH, "Low water!");
        current_state = SYSTEM_ERROR; // Transition to ERROR
        *PORT_BUTTONS &= ~FAN_CONTROL_MASK; // Turn off the fan in ERROR
      } else if ((int)dht_sensor.readTemperature(true) < temperature_threshold) {
        current_state = SYSTEM_IDLE;  // Transition to IDLE if temperature drops below threshold
        *PORT_BUTTONS &= ~FAN_CONTROL_MASK; // Ensure fan is off in IDLE
      }
      break;
  }

  // Update LEDs based on the current state
  update_led_state(current_state);

  // Handle control button (CTRL_BTN) for stepper motor
  if ((*PIN_BUTTONS & CONTROL_BUTTON_MASK) == 0) {  // If CONTROL_BUTTON is pressed (LOW)
    unsigned char uart_message[64];
    snprintf(uart_message, 64, "\nVENT POSITION UPDATED\n");
    send_uart_message(uart_message, strlen(uart_message));  // Send message over UART
    stepper_motor.step(1);  // Move stepper motor by one step
  }

  // Handle state transitions and log changes
  if (previous_state != current_state) {
    unsigned char uart_message[256];
    snprintf(uart_message, 256, "\nSTATE TRANSITION: %s -> %s", state_labels[previous_state], state_labels[current_state]);
    send_uart_message(uart_message, strlen(uart_message));  // Log state transition
    snprintf(uart_message, 256, "\nCurrent Time: %02d:%02d:%02d\nCurrent Date: %d/%d/%d\n", \
            current_time.hour(), current_time.minute(), current_time.second(), current_time.day(), current_time.month(), current_time.year());
    send_uart_message(uart_message, strlen(uart_message));  // Log current time and date
    lcd.clear();  // Clear the LCD for updated display
  }

  // Update the previous state
  previous_state = current_state;
}

void update_led_state(SYSTEM_STATE state) {
  *PORT_LEDS &= ~(GREEN_LED_MASK | YELLOW_LED_MASK | RED_LED_MASK | BLUE_LED_MASK); // Clear all LEDs
  *PORT_LEDS |= led_state_masks[state]; // Set the LED corresponding to the current state
}

ISR(PCINT0_vect) {
  if (*PIN_BUTTONS & RESET_BUTTON_MASK) { // RESET button pressed
    if (current_state == SYSTEM_ERROR || current_state == SYSTEM_RUNNING) {
      current_state = SYSTEM_IDLE;  // Reset to IDLE from ERROR or RUNNING
    }
  } else if (*PIN_BUTTONS & START_BUTTON_MASK) {  // START button pressed
    if (current_state == SYSTEM_RUNNING || current_state == SYSTEM_IDLE || current_state == SYSTEM_ERROR) {
      current_state = SYSTEM_DISABLED;  // Transition to DISABLED
    } else if (current_state == SYSTEM_DISABLED) {
      current_state = SYSTEM_IDLE;  // Transition to IDLE
    }
  }
}

void refresh_temperature_humidity(char *buffer) {
  int humidity = (int)dht_sensor.readHumidity();
  int temperature = (int)dht_sensor.readTemperature(true);

  snprintf(buffer, LCD_MAX_LENGTH, "H:%d T:%dF", humidity, temperature);
}

void initialize_io() {
  *DDR_LEDS |= GREEN_LED_MASK;
  *DDR_LEDS |= YELLOW_LED_MASK;
  *DDR_LEDS |= RED_LED_MASK;
  *DDR_LEDS |= BLUE_LED_MASK;
  *DDR_BUTTONS |= FAN_CONTROL_MASK;

  *PORT_BUTTONS |= START_BUTTON_MASK;
  *DDR_BUTTONS &= ~(START_BUTTON_MASK);

  *PORT_BUTTONS |= RESET_BUTTON_MASK;
  *DDR_BUTTONS &= ~(RESET_BUTTON_MASK);

  *PORT_BUTTONS |= CONTROL_BUTTON_MASK;
  *DDR_BUTTONS &= ~(CONTROL_BUTTON_MASK);

  *PC_INTERRUPT_CONTROL_REGISTER |= 0x01;
  *PC_INTERRUPT_MASK_REGISTER |= PIN_CHANGE_INTERRUPT_MASK;
}

void initialize_adc() {
  *ADC_CONTROL_STATUS_A |= 0b10000000;
  *ADC_CONTROL_STATUS_A &= 0b11011111;
  *ADC_CONTROL_STATUS_B &= 0b11110111;
  *ADC_CONTROL_STATUS_B &= 0b11111000;
  *ADC_MUX_REGISTER &= 0b01111111;
  *ADC_MUX_REGISTER |= 0b01000000;
  *ADC_MUX_REGISTER &= 0b11011111;
  *ADC_MUX_REGISTER &= 0b11100000;
}

unsigned int read_adc(unsigned char adc_channel) {
  *ADC_MUX_REGISTER &= 0b11100000;
  *ADC_CONTROL_STATUS_B &= 0b11110111;

  if (adc_channel > 7) {
    adc_channel -= 8;
    *ADC_CONTROL_STATUS_B |= 0b00001000;
  }

  *ADC_MUX_REGISTER += adc_channel;
  *ADC_CONTROL_STATUS_A |= 0x40;

  while ((*ADC_CONTROL_STATUS_A & 0x40) != 0);

  return *ADC_DATA_REGISTER;
}

void initialize_uart(unsigned long baud_rate) {
  unsigned long cpu_frequency = 16000000;
  unsigned int baud_setting = (cpu_frequency / 16 / baud_rate - 1);

  *UART_CONTROL_STATUS_A = 0x20;
  *UART_CONTROL_STATUS_B = 0x18;
  *UART_CONTROL_STATUS_C = 0x06;
  *UART_BAUD_RATE_REGISTER = baud_setting;
}

void send_uart_char(unsigned char character) {
  while ((*UART_CONTROL_STATUS_A & TRANSMIT_BUFFER_EMPTY) == 0) {};
  *UART_DATA_REGISTER = character;
}

void send_uart_message(unsigned char *string, int length) {
  while ((*UART_CONTROL_STATUS_A & TRANSMIT_BUFFER_EMPTY) == 0) {}; // Wait until Transmit Buffer is Empty
  for (int i = 0; i < length && string[i] != '\0'; i++) {
    while ((*UART_CONTROL_STATUS_A & TRANSMIT_BUFFER_EMPTY) == 0) {}; // Wait for buffer again before sending each character
    *UART_DATA_REGISTER = string[i]; // Send the character
  }
}
