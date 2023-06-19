// https://www.hackster.io/pradeeplogu0/real-time-gps-monitoring-with-qubitro-and-m5stickc-a2bc7c

// https://github.com/mikalhart/TinyGPSPlus/blob/master/README.md
// http://arduiniana.org/libraries/tinygpsplus/

// Course to back left corner of garden at compost bin Lat = 51.39126, long=-0.28764
//
//possible fix to deepSleep with timer #31 - https://github.com/m5stack/M5StickC-Plus/pull/31
//Sleep causing unresponsive device #13 https://github.com/m5stack/M5StickC-Plus/issues/13
//AXP192.cpp SetSleep() is different than the one for M5StickC #1 https://github.com/m5stack/M5StickC-Plus/issues/1
#include <M5StickCPlus.h>

// rename the git file "mercator_secrets_template.c" to the filename below, filling in your wifi credentials etc.
#include "mercator_secrets.c"

#include <esp_now.h>

uint16_t ESPNowMessagesDelivered = 0;
uint16_t ESPNowMessagesFailedToDeliver = 0;

uint8_t ESPNow_data_to_send = 33;

esp_now_peer_info_t ESPNow_slave;
const uint8_t ESPNOW_CHANNEL = 1;
const uint8_t ESPNOW_PRINTSCANRESULTS = 0;
const uint8_t ESPNOW_DELETEBEFOREPAIR = 0;

const uint8_t SILKY_ESPNOW_COMMAND_NEXT_TRACK = 0;
const uint8_t SILKY_ESPNOW_COMMAND_TOGGLE_PLAYBACK = 1;
const uint8_t SILKY_ESPNOW_COMMAND_CYCLE_VOLUME_UP = 2;


#include "tb_display.h"

// screen Rotation values:
// 1 = Button right
// 2 = Button above
// 3 = Button left
// 4 = Button below

uint8_t tb_buffer_screen_orientation = 3;
uint8_t tb_buffer_chosenTextSize = 2;

#include <TinyGPS++.h>

// OTA updates start
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

const int SCREEN_LENGTH = 240;
const int SCREEN_WIDTH = 135;

const int UPLINK_BAUD_RATE = 9600;

enum e_display_brightness {OFF_DISPLAY = 7, DIM_DISPLAY = 8, HALF_BRIGHT_DISPLAY = 10, BRIGHTEST_DISPLAY = 14};
enum e_uplinkMode {SEND_NO_UPLINK_MSG, SEND_TEST_UPLINK_MSG, SEND_BASIC_UPLINK_MSG, SEND_BASIC_WITH_DEPTH_UPLINK_MSG, SEND_FULL_UPLINK_MSG};

void sendNoUplinkTelemetryMessages();
void sendUplinkTestMessage();
void sendBasicUplinkTelemetryMessage();
void sendBasicWithDepthUplinkTelemetryMessage();
void sendFullUplinkTelemetryMessage();

// feature switches

const e_display_brightness ScreenBrightness = BRIGHTEST_DISPLAY;
const e_uplinkMode uplinkMode = SEND_FULL_UPLINK_MSG;
bool enableUplinkComms = true;  // can be toggled through UI
const bool enableTiltCompensation = true;
const bool enableDigitalCompass = true;
const bool enableHumiditySensor = true;
const bool enableDepthSensor = true;
const bool enableIMUSensor = true;
const bool enableNavigationGraphics = true;
const bool enableNavigationTargeting = true;
const bool enableRecentCourseCalculation = true;
bool enableGlobalUptimeDisplay = false;    // adds a timer to compass heading display so can see if a crash/reboot has happened
bool enableWifiAtStartup = true;

bool otaActiveListening = true; // OTA updates toggle
bool otaFirstInit = false;       // Start OTA at boot

bool enableESPNow = false;       //
bool ESPNowActive = false;       // can be toggled through interface.

void OnESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

// compilation switches

//#define INCLUDE_TWITTER_AT_COMPILE_TIME
//#define INCLUDE_SMTP_AT_COMPILE_TIME
//#define INCLUDE_QUBITRO_AT_COMPILE_TIME

void (*fp_sendUplinkMessage)() =  ( uplinkMode == SEND_NO_UPLINK_MSG ? &sendNoUplinkTelemetryMessages :
                                  ( uplinkMode == SEND_TEST_UPLINK_MSG ? &sendUplinkTestMessage :
                                  ( uplinkMode == SEND_BASIC_UPLINK_MSG ? &sendBasicUplinkTelemetryMessage :
                                  ( uplinkMode == SEND_BASIC_WITH_DEPTH_UPLINK_MSG ? &sendBasicWithDepthUplinkTelemetryMessage :
                                  ( uplinkMode == SEND_FULL_UPLINK_MSG ? &sendFullUplinkTelemetryMessage : &sendNoUplinkTelemetryMessages)))));

#ifdef INCLUDE_TWITTER_AT_COMPILE_TIME
  // TWITTER START
    #include <WiFiClientSecure.h>   // Twitter
    #include <TweESP32.h>          // Install from Github - https://github.com/witnessmenow/TweESP32
    #include <TwitterServerCert.h> // included with above
    // Dependant Libraries
    #include <UrlEncode.h> //Install from library manager
    #include <ArduinoJson.h> //Install from library manager
    bool connectToTwitter = true;
    WiFiClientSecure secureTwitterClient;
    TweESP32 twitter(secureTwitterClient, twitterConsumerKey, twitterConsumerSecret, twitterAccessToken, twitterAccessTokenSecret, twitterBearerToken);
  // TWITTER END
#endif

// flag sent to float if tweet requested (does not require twitter libs locally)
bool setTweetLocationNowFlag = false;
bool setTweetEmergencyNowFlag = false;

#ifdef INCLUDE_SMTP_AT_COMPILE_TIME
  // SMTP Email START
    #include <ESP_Mail_Client.h>
    SMTPSession smtp;
    bool connectToSMTP = true;
  // SMTP Email END
#endif

#ifdef INCLUDE_QUBITRO_AT_COMPILE_TIME
  // QUBITRO START
    #include <QubitroMqttClient.h>
    const bool connectToQubitro = true;
    float qubitro_upload_duty_ms = 0;
    uint32_t last_qubitro_upload = 0;
    
    WiFiClient wifiClient;
    QubitroMqttClient qubitro_mqttClient(wifiClient);
  // QUBITRO END
#endif

const uint8_t telemetry_send_full_message_duty_cycle = 1;
uint8_t telemetry_message_count = 0;

const uint32_t console_screen_refresh_minimum_interval = 500; // milliseconds
uint32_t lastConsoleScreenRefresh = 0;
bool requestConsoleScreenRefresh = false;

#define USB_SERIAL Serial

const char* ssid_not_connected = "-";
const char* ssid_connected = ssid_not_connected;

const bool enableOTAServer = true; // OTA updates
AsyncWebServer asyncWebServer(80);
// OTA updates end

uint32_t showLatLongStartTime = 0;
const uint32_t showLatLongHoldDuration = 5000;

char uplink_preamble_pattern[] = "MBJAEJ";
char uplinkTestMessages[][6] = {"MSG0 ", "MSG1 ", "MSG2 ", "MSG3 "};
char newWayMarkerLabel[2];
char directionMetricLabel[2];

// I2C and framework
#include <Wire.h>                   // I2C framework
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>       // Magnetometer
#include <Adafruit_LSM303_Accel.h>  // Accelerometer
//#include <Adafruit_AHTX0.h>         // Temp and Humidity Sensor Adafruit AHT20 - now in float
#include <Adafruit_BME280.h>
//#include <Adafruit_APDS9960.h>
#include "MS5837.h" // water pressure sensor

const uint16_t SEALEVELPRESSURE_HPA = 1000.00;

#include <math.h>

enum gpsStatus {GPS_NO_GPS_LIVE_IN_FLOAT, GPS_NO_FIX_FROM_FLOAT, GPS_FIX_FROM_FLOAT};
gpsStatus GPS_status = GPS_NO_GPS_LIVE_IN_FLOAT;

// lat/long of the compost bin in the garden
double ref_lat = 51.391231;
double ref_lng = -0.287616;

uint16_t telemetryMessage[100];

char shortBlackOut[] = "BL";
char shortAntiClockwise[] = "AC";
char shortAhead[] = "AH";
char shortClockwise[] = "CL";
char shortTurnAround[] = "TA";
char shortUnknownMarker[] = "UM";
char shortUndefinedMarker[] = "UD";

char shortCompassHeadingDirectionMetric[] ="CH";
char shortJourneyCourseDirectionMetric[] ="JC";
char shortUndefinedDirectionMetric[] ="JC";

char displayLabel[] = "??";

char navCompassDisplayLabel[] = "CM";
char navCourseDisplayLabel[] = "CO";
char locationDisplayLabel[] = "LO";
char journeyDisplayLabel[] = "JO";
char showLatLongDisplayLabel[] = "LL";
char audioTestDisplayLabel[] = "AT";
char undefinedDisplayLabel[] = "??";

class navigationTarget
{
    static const uint8_t maxLabelLength = 24;

  public:

    char*  _label;
    float _lat;
    float _long;

    navigationTarget()
    {
      _label[0] = NULL;
      _lat = _long = 0.0;
    }

    navigationTarget(char*  label, float latitude, float longitude) : _label(label), _lat(latitude), _long(longitude)
    {
    }
};
const uint8_t targetCount = 10;

navigationTarget diveOneTargets[targetCount] =
{
  [0] = { ._label = "Confined\nArea", ._lat = 51.459987, ._long = -0.548600},
  [1] = { ._label = "Bus", ._lat = 51.460073, ._long = -0.548515},
  [2] = { ._label = "Ship\nContainer", ._lat = 51.460014, ._long = -0.548735},
  [3] = { ._label = "Cotton\nReel",  ._lat = 51.461496, ._long = -0.549753},
  [4] = { ._label = "The\nSource", ._lat = 51.462418, ._long = -0.549491},
  [5] = { ._label = "Mystery?", ._lat = 51.459745, ._long = -0.546649},   // plane?
  [6] = { ._label = "Buttie\nJetty", ._lat = 51.460015, ._long = -0.548316},    // jetty near dive centre
  [7] = { ._label = "Mid\nJetty", ._lat = 51.459547, ._long = -0.547461},
  [8] = { ._label = "Old\nJetty",  ._lat = 51.459280, ._long = -0.547084},
  [9] = { ._label = "Mark's\nGaff", ._lat = 51.391231, ._long = -0.287616}
};

navigationTarget* nextTarget = diveOneTargets;

enum e_way_marker {BLACKOUT_MARKER, GO_ANTICLOCKWISE_MARKER, GO_AHEAD_MARKER, GO_CLOCKWISE_MARKER, GO_TURN_AROUND_MARKER, UNKNOWN_MARKER};
enum e_direction_metric {JOURNEY_COURSE, COMPASS_HEADING};

double heading_to_target = 0, distance_to_target = -0;
double journey_lat = 0, journey_lng = 0, journey_course = 0, journey_distance = 0;
float magnetic_heading = 0;
float mag_accel_x = 0, mag_accel_y = 0, mag_accel_z = 0;
float mag_tesla_x = 0, mag_tesla_y = 0, mag_tesla_z = 0;
float humidity = 0, temperature = 0, air_pressure = 0, pressure_altitude = 0, depth = 0, water_temperature = 0, water_pressure = 0, depth_altitude = 0;
uint16_t red_sensor = 0, green_sensor = 0, blue_sensor = 0, clear_sensor = 0;
uint8_t gesture = 255, proximity = 255;
char gesture_symbol = '-';
const float pressure_correction = 0;  // mbar, calibrated against Braggs Wunderground - not used now
const float depth_correction = 0;

const uint32_t journey_calc_period = 500;    // in milliseconds
const uint32_t journey_min_dist = 0;          // in metres

//const uint32_t journey_calc_period = 10000;    // in milliseconds
//const uint32_t journey_min_dist = 5;          // in metres

uint32_t last_journey_commit_time = 0;
uint32_t journey_clear_period = 15000;    // clear the journey info after 15 secs inactivity
uint32_t lastWayMarkerChangeTimestamp = 0;
e_way_marker lastWayMarker = BLACKOUT_MARKER;    // SHOULD BE ENUM
e_way_marker newWayMarker = BLACKOUT_MARKER;     // SHOULD BE ENUM
bool blackout_journey_no_movement = true;
uint8_t activity_count = 0;
void refreshDirectionGraphic(float directionOfTravel, float headingToTarget);

e_direction_metric directionMetric = COMPASS_HEADING;

uint32_t fixCount = 0;
uint32_t newPassedChecksum = 0;
uint32_t newFailedChecksum = 0;
uint32_t uplinkMessageCount = 0;
uint32_t passedChecksumCount = 0;

uint8_t graphicsCount = 0;

char activity_indicator[] = "\\|/-";

enum e_mako_displays {NAV_COMPASS_DISPLAY, NAV_COURSE_DISPLAY, LOCATION_DISPLAY, JOURNEY_DISPLAY, SHOW_LAT_LONG_DISPLAY, AUDIO_TEST_DISPLAY};
const e_mako_displays first_display_rotation = NAV_COMPASS_DISPLAY;
const e_mako_displays last_display_rotation = JOURNEY_DISPLAY;

e_mako_displays display_to_show = first_display_rotation;
e_mako_displays previous_display_shown = first_display_rotation;

void switchToNextDisplayToShow()
{
  if ((int)display_to_show > (int)last_display_rotation)
  {
    display_to_show = previous_display_shown;
  }
  else
  {
    display_to_show = (e_mako_displays)((int)display_to_show + 1);

    if (display_to_show > last_display_rotation)
      display_to_show = first_display_rotation;
  }

  M5.Lcd.fillScreen(TFT_BLACK);
  requestConsoleScreenRefresh=true;
}

const uint8_t RED_LED_GPIO = 10;

const bool writeLogToSerial = false;

TinyGPSPlus gps;
int uart_number = 2;
HardwareSerial float_serial(uart_number);   // UART number 2: This uses Grove SCL=GPIO33 and SDA=GPIO32 for Hardware UART Tx and Rx
double Lat, Lng;
String  lat_str , lng_str;
int satellites = 0;
double b, c = 0;
int power_up_no_fix_byte_loop_count = 0;

hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t TimerCount = 0;

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  //    digitalWrite(RED_LED_GPIO, TimerCount % 10 );
  TimerCount++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

bool useGrovePortForGPS = false;

uint8_t nextUplinkMessage = 0;

Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
// Adafruit_AHTX0 Adafruit_TempHumidity; // now in float
Adafruit_BME280 Adafruit_TempHumidityPressure;
// Adafruit_APDS9960 Adafruit_GestureSensor;

MS5837 BlueRobotics_DepthSensor;

const uint8_t GESTURE_INT_PIN = 3;

template <typename T> struct vector
{
  T x, y, z;
};

// Stores min and max magnetometer values from calibration
vector<float> magnetometer_max;
vector<float> magnetometer_min;
vector<float> magnetometer_vector, accelerometer_vector;
vector<float> imu_gyro_vector, imu_lin_acc_vector, imu_rot_acc_vector;
float imu_temperature = 0.0;

void getM5ImuSensorData(float* gyro_x, float* gyro_y, float* gyro_z,
                        float* lin_acc_x, float* lin_acc_y, float* lin_acc_z,
                        float* rot_acc_x, float* rot_acc_y, float* rot_acc_z,
                        float* IMU_temperature)
{
  if (enableIMUSensor)
  {
    M5.IMU.getGyroData(gyro_x, gyro_y, gyro_z);
    M5.IMU.getAccelData(lin_acc_x, lin_acc_y, lin_acc_z);
    M5.IMU.getAhrsData(rot_acc_x, rot_acc_y, rot_acc_z);
    M5.IMU.getTempData(IMU_temperature);
  }
  else
  {
    *gyro_x = *gyro_y = *gyro_z = *lin_acc_x = *lin_acc_y = *lin_acc_z = *rot_acc_x = *rot_acc_y = *rot_acc_z = *IMU_temperature = 0.1;
  }
}


bool compassAvailable = true;
bool humidityAvailable = true;
bool gestureAvailable = true;
bool depthAvailable = true;
bool imuAvailable = true;


// Magnetic Compass averaging and refresh rate control
const uint8_t s_smoothCompassBufferSize = 20;
float s_smoothedCompassHeading[s_smoothCompassBufferSize];
uint8_t s_nextCompassSampleIndex = 0;
bool s_compassBufferInitialised = false;
const uint16_t s_compassSampleDelay = 50;
uint32_t s_lastCompassDisplayRefresh = 0;
const uint32_t s_compassHeadingUpdateRate = 200; // time between each compass update to screen
float s_lastDisplayedCompassHeading = 0.0;

uint32_t s_lastTempHumidityDisplayRefresh = 0;
const uint32_t s_tempHumidityUpdateRate = 1000; // time between each compass update to screen

const uint8_t BUTTON_GOPRO_TOP_PIN = 25;
const uint8_t BUTTON_GOPRO_SIDE_PIN = 0;
const uint32_t MERCATOR_DEBOUNCE_MS = 0;

const uint8_t GROVE_GPS_RX_PIN = 33;
const uint8_t GROVE_GPS_TX_PIN = 32;

const uint8_t HAT_GPS_RX_PIN = 26;
const uint8_t IR_LED_GPS_TX_PIN = 9;


Button BtnGoProTop = Button(BUTTON_GOPRO_TOP_PIN, true, MERCATOR_DEBOUNCE_MS);    // from utility/Button.h for M5 Stick C Plus
Button BtnGoProSide = Button(BUTTON_GOPRO_SIDE_PIN, true, MERCATOR_DEBOUNCE_MS); // from utility/Button.h for M5 Stick C Plus
uint16_t sideCount = 0, topCount = 0;

Button* p_primaryButton = NULL;
Button* p_secondButton = NULL;
void updateButtonsAndBuzzer();

const uint32_t CLEARED_FIX_TIME_STAMP = 0xF0000000;
uint32_t latestFixTimeStamp = CLEARED_FIX_TIME_STAMP;

// Magnetic heading calculation functions
template <typename T> float magHeading(vector<T> from);
template <typename Ta, typename Tb, typename To> void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
template <typename Ta, typename Tb> float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
void vector_normalize(vector<float> *a);
bool getMagHeadingTiltCompensated(float& tiltCompensatedHeading);
bool getMagHeadingNotTiltCompensated(float& heading);
bool getSmoothedMagHeading(float& b);
std::string getCardinal(float b);
//void getTempAndHumidityAHT20(float& h, float& t);

void getTempAndHumidityAndAirPressureBME280(float& h, float& t, float& p, float& p_a);

// depth in metres, temperature in C, water pressure in Bar, Altitude in m
void getDepth(float& d, float& d_t, float& d_p, float& d_a);

void testForDualButtonPressAutoShutdownChange();

void goBlackout();
void goAhead();
void goClockwise();
void goUnknown();
void goAntiClockwise();
void goTurnAround();

void drawGoAhead(const bool show);
void drawGoBlackout(const bool show);
void drawGoAhead(const bool show);
void drawGoClockwise(const bool show);
void drawGoTurnAround(const bool show);
void drawGoUnknown(const bool show);
void drawGoAntiClockwise(const bool show);

const float minimumUSBVoltage = 2.0;
long USBVoltageDropTime = 0;
long milliSecondsToWaitForShutDown = 500;
bool autoShutdownOnNoUSBPower = true;

bool enableButtonTestMode = false;

bool goProButtonsPrimaryControl = true;

void readAndTestGoProButtons();

void shutdownIfUSBPowerOff();

void refreshDepthDisplay();

float hallOffset = 0;  // Store the initial value of magnetic force
const float magnetHallReadingForReset = -50;

float ESP32_hallRead();
bool isMagnetPresentHallSensor();

bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout);
bool connectWiFiNoOTA(const char* _ssid, const char* _password, const char* label, uint32_t timeout);

void updateButtonsAndBuzzer()
{
  p_primaryButton->read();
  p_secondButton->read();
  M5.Beep.update();
}

void flashNextTargetOnScreen(const bool continueOnCourse)
{
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(0, 5);
  if (continueOnCourse)
    M5.Lcd.printf ("Continue to:\n\n%s", nextTarget->_label);
  else
    M5.Lcd.printf ("Goto next:\n\n%s", nextTarget->_label);
  delay(2500);
  M5.Lcd.fillScreen(TFT_BLACK);
}


void setup()
{
  M5.begin();

  M5.Lcd.setTextSize(tb_buffer_chosenTextSize);

  tb_display_init(tb_buffer_screen_orientation, M5.Lcd.textsize); // MBJMBJ - change this
  tb_display_print_String("Mercator Origins - Text Buffer Enabled\n");
  delay(1000);

  if (enableIMUSensor)
  {
    M5.Imu.Init();
  }
  else
  {
    USB_SERIAL.println("IMU Sensor Off");
    M5.Lcd.println("IMU Sensor Off");
    imuAvailable = false;
  }

  M5.Axp.ScreenBreath(ScreenBrightness);

  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);

  if (goProButtonsPrimaryControl)
  {
    p_primaryButton = &BtnGoProTop;
    p_secondButton = &BtnGoProSide;
  }
  else
  {
    p_primaryButton = &M5.BtnA;
    p_secondButton = &M5.BtnB;
  }

  if (enableOTAServer && enableWifiAtStartup)
  {
    if (!setupOTAWebServer(ssid_1, password_1, label_1, timeout_1))
      if (!setupOTAWebServer(ssid_2, password_2, label_2, timeout_2))
        setupOTAWebServer(ssid_3, password_3, label_3, timeout_3);

    if (WiFi.status() == WL_CONNECTED)
      otaFirstInit = true;
  }

  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html
  //  uart_set_mode(uart_number, UART_MODE_RS485_HALF_DUPLEX);

  // settings from M5 with no magnets - X arrow on magnetometer pointing in direction of heading.
  magnetometer_min = (vector<float>) {
    -51.15, -60.45, 0.00
  };
  magnetometer_max = (vector<float>) {
    53.10, 37.05, 110.25
  };

  //  if (! Adafruit_TempHumidity.begin()) {
  //    USB_SERIAL.println("Could not find AHT? Check wiring");
  //    humidityAvailable=false;
  //  }

  M5.Lcd.setCursor(0, 0);

  if (enableHumiditySensor)
  {
    if (!Adafruit_TempHumidityPressure.begin())
    {
      USB_SERIAL.println("Could not find BME280 Barometer");
      M5.Lcd.println("BE280 T/H/P bad");
      delay(5000);
      humidityAvailable = false;
    }
  }
  else
  {
    USB_SERIAL.println("BME280 Humidity Off");
    M5.Lcd.println("BME280 Humidity Off");
    humidityAvailable = false;
    temperature = 0.1;
    humidity = 0.1;
    air_pressure = 0.1;
  }

  if (enableDigitalCompass)
  {
    if (!mag.begin())
    {
      USB_SERIAL.println("Could not find LIS2MDL Magnetometer. Check wiring");
      M5.Lcd.println("LIS2MDL Magnetometer bad");
      delay(5000);
      compassAvailable = false;
    }
  }
  else
  {
    USB_SERIAL.println("LSM303 Compass off");
    M5.Lcd.println("LSM303 Compass off");
    compassAvailable = false;
  }

  if (enableDigitalCompass)
  {
    if (!accel.begin())
    {
      USB_SERIAL.println("Unable to initialize LSM303 accelerometer");
      M5.Lcd.println("LSM303 accelerometer bad");
      compassAvailable = false;
    }
  }
  else
  {
    USB_SERIAL.println("LSM303 Accel off");
    M5.Lcd.println("LSM303 Accel off");
    compassAvailable = false;
  }



  /*
    if (!Adafruit_GestureSensor.begin())
    {
      USB_SERIAL.println("Unable to init APDS9960 gesture");
      M5.Lcd.println("APDS9960 gesture bad");
      gestureAvailable=false;
    }
    else
    {
      Adafruit_GestureSensor.enableColor(true);
    //    Adafruit_GestureSensor.enableProximity(true);
    //    Adafruit_GestureSensor.enableGesture(true);

      //set the interrupt threshold to fire when proximity reading goes above 175
    //    Adafruit_GestureSensor.setProximityInterruptThreshold(0, 175);

      //enable the proximity interrupt
    //    Adafruit_GestureSensor.enableProximityInterrupt();
    }
  */
  // do depth centre initialisation here
  if (enableDepthSensor)
  {
    if (!BlueRobotics_DepthSensor.init())
    {
      USB_SERIAL.println("Could not init depth sensor");
      M5.Lcd.println("Could not init depth sensor");
      depthAvailable = false;
    }
  }
  else
  {
    USB_SERIAL.println("Depth Sensor Off");
    M5.Lcd.println("Depth Sensor Off");
    depthAvailable = false;
    depth = 0;
  }

  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setCursor(0, 0);

  if (useGrovePortForGPS)
  {
    float_serial.begin(UPLINK_BAUD_RATE, SERIAL_8N1, GROVE_GPS_RX_PIN, GROVE_GPS_TX_PIN);   // pin 33=rx (white M5), pin 32=tx (yellow M5), specifies the grove SCL/SDA pins for Rx/Tx
    float_serial.setRxBufferSize(256);
  }
  else
  {
    float_serial.begin(UPLINK_BAUD_RATE, SERIAL_8N1, HAT_GPS_RX_PIN, IR_LED_GPS_TX_PIN);   // pin 26=rx, 9=tx specifies the HAT pin for Rx and the IR LED for Tx (not used)
    float_serial.setRxBufferSize(256);
  }
  updateButtonsAndBuzzer();
  // cannot use Pin 0 for receive of GPS (resets on startup), can use Pin 36, can use 26
  // cannot use Pin 0 for transmit of GPS (resets on startup), only Pin 26 can be used for transmit.

  /*
     Hi, I would like to use the GPIO 25 and 26 pins in the header for I2C, so I am setting them up as follows in MicroPython:

    from machine import Pin, I2C
    i2c = I2C(1, scl=Pin(26), sda=Pin(25), freq=400000)

    However, the description says that "G36/G25 share the same port, when one of the pins is used, the other pin should be set as a floating input" and
    provides the following example
    ]
    For example, to use the G36 pin as the ADC input, Configuration the G25 pin as FLOATING
    setup()
    {
    M5.begin();
    pinMode(36, INPUT);
    gpio_pulldown_dis(GPIO_NUM_25);
    gpio_pullup_dis(GPIO_NUM_25);
    }

    It seems that I should set GPIO36 to Floating.
  */

  // see https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareUSB_SERIAL.h
  // see https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareUSB_SERIAL.cpp

  // https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
  pinMode(RED_LED_GPIO, OUTPUT); // Red LED
  digitalWrite(RED_LED_GPIO, HIGH); // switch off

  /*
      timerSemaphore = xSemaphoreCreateBinary();
      timer = timerBegin(0, 80, true);      // pre-scaler of 80, dividing 80MHz by 80 to trigger every 1uS
      timerAttachInterrupt(timer, &onTimer, true);
      timerAlarmWrite(timer, 50000, true);  // interupt generated every 50,000 uS = 5s
      timerAlarmEnable(timer);
  */
  //    SendCASICNavXQuery(float_serial);
  //    waitForCASICNavXResponse(float_serial);
  //    waitForCASICACKResponse(float_serial);

#ifdef INCLUDE_TWITTER_AT_COMPILE_TIME
  if (connectToTwitter && WiFi.status() == WL_CONNECTED)
  {
    //Required for Oauth for sending tweets
    twitter.timeConfig();
    // Checking the cert is the best way on an ESP32i
    // This will verify the server is trusted.
    secureTwitterClient.setCACert(twitter_server_cert);

    // send test tweet
    bool success = twitter.sendTweet("Test Tweet From Mercator Origins - Bluepad Labs. The Scuba Hacker");
  }
#endif

#ifdef INCLUDE_SMTP_AT_COMPILE_TIME
  if (connectToSMTP && WiFi.status() == WL_CONNECTED)
  {
    sendTestByEmail();
  }
#endif


#ifdef INCLUDE_QUBITRO_AT_COMPILE_TIME
  if (connectToQubitro && WiFi.status() == WL_CONNECTED)
  {
    qubitro_connect();
  }
#endif

}

void loop_no_gps()
{
  M5.Axp.ScreenBreath(10);             // 8 mid brightness

  if (autoShutdownOnNoUSBPower)
    shutdownIfUSBPowerOff();

  getTempAndHumidityAndAirPressureBME280(humidity, temperature, air_pressure, pressure_altitude);

  getDepth(depth, water_temperature, water_pressure, depth_altitude);

  getMagHeadingTiltCompensated(magnetic_heading);

  //  uint16_t r, g, b, c;

  //wait for color data to be ready
  //  while(!Adafruit_GestureSensor.colorDataReady())
  //  delay(5);

  //  Adafruit_GestureSensor.getColorData(&red_sensor, &green_sensor, &blue_sensor, &clear_sensor);

  /*
    gesture = Adafruit_GestureSensor.readGesture(); // APDS9960_UP, APDS9960_DOWN, APDS9960_LEFT, APDS9960_RIGHT

    if(gesture == APDS9960_DOWN)  gesture_symbol='v';
    if(gesture == APDS9960_UP)    gesture_symbol='^';
    if(gesture == APDS9960_LEFT)  gesture_symbol='<';
    if(gesture == APDS9960_RIGHT) gesture_symbol='>';

    //print the proximity reading when the interrupt pin goes low
    if(!digitalRead(GESTURE_INT_PIN)){
      Serial.println(Adafruit_GestureSensor.readProximity());

      //clear the interrupt
      Adafruit_GestureSensor.clearInterrupt();
    }
  */

  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(4);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setCursor(0, 0);

  M5.Lcd.printf("  %.2f m  \n", depth); // was gesture char

  M5.Lcd.setTextSize(1);
  M5.Lcd.println("");

  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("%.1fC   %.1f%%\n", temperature, humidity);
  M5.Lcd.setTextSize(1);
  M5.Lcd.println("");

  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLUE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf(" %.1f mBar \n", air_pressure);
  M5.Lcd.setTextSize(2);
  M5.Lcd.println("");

  M5.Lcd.setTextColor(TFT_YELLOW, TFT_MAGENTA);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("%.2fV %.1fmA \n", M5.Axp.GetBatVoltage(), M5.Axp.GetBatCurrent());

  //  testForShake();

  if (autoShutdownOnNoUSBPower)
  {
    shutdownIfUSBPowerOff();
  }

  sleep(250);
}

void loop()
{
  if (autoShutdownOnNoUSBPower)
    shutdownIfUSBPowerOff();

  /*
    testForDualButtonPressAutoShutdownChange();

    if (!autoShutdownOnNoUSBPower && gps.location.isValid() == false)
    {
      // if disabling of 0V USB volts happens when no GPS, switch into sensor only mode
      // if no gps / no cable connected and autoshutdown is disabled then show the sensor stats only
      loop_no_gps();
      return;
    }
  */

  bool msgProcessed = processGPSMessageIfAvailable();
  
  if (!msgProcessed)
  {
    // no gps message received, do a manual refresh of sensors and screen
    acquireAllSensorReadings(); // compass, IMU, Depth, Temp, Humidity, Pressure

    if (millis() > lastConsoleScreenRefresh + console_screen_refresh_minimum_interval)
    {
      refreshConsoleScreen();
      lastConsoleScreenRefresh = millis();
    }
  }
  else
  {
     lastConsoleScreenRefresh = millis();
  }

  checkForButtonPresses();

  if (requestConsoleScreenRefresh)
  {
    requestConsoleScreenRefresh = false;
    refreshConsoleScreen();
    lastConsoleScreenRefresh = millis();
  }

  refreshGlobalStatusDisplay();

#ifdef INCLUDE_QUBITRO_AT_COMPILE_TIME
// This isn't included in the build or enabled in production.
  if (connectToQubitro)
    uploadTelemetryToQubitro();
#endif
}



bool processGPSMessageIfAvailable()
{ 
  bool result = (float_serial.available() > 0);

  while (float_serial.available() > 0)
  {
    if (enableButtonTestMode)
      readAndTestGoProButtons();

    //   testForDualButtonPressAutoShutdownChange();

    char nextByte = float_serial.read();

    if (writeLogToSerial)
      USB_SERIAL.write(nextByte);

    if (gps.encode(nextByte))
    {
      if (gps.location.isValid())
      {
        uint32_t newFixCount = gps.sentencesWithFix();
        newPassedChecksum = gps.passedChecksum();
        newFailedChecksum = gps.failedChecksum();
        if (newFixCount > fixCount)
        {
          fixCount = newFixCount;

          if (writeLogToSerial)
          {
            digitalWrite(RED_LED_GPIO, fixCount % 2);
            USB_SERIAL.printf("\nFix: %lu Good Msg: %lu Bad Msg: %lu", fixCount, newPassedChecksum, gps.failedChecksum());
          }

          latestFixTimeStamp = millis();
        }

        if (power_up_no_fix_byte_loop_count > -1)
        {
          // clear the onscreen counter that increments whilst attempting to get first valid location
          power_up_no_fix_byte_loop_count = -1;
          M5.Lcd.fillScreen(TFT_BLACK);
        }

        if (newPassedChecksum <= passedChecksumCount)
        {
          // incomplete message received, continue reading bytes, don't update display.
          // continue reading bytes, 
//          return false;     // this was a straight return to terminate the loop() function before.
        }
        else
        {
          passedChecksumCount = newPassedChecksum;

          // At this point a new lat/long fix has been received and is available.
          refreshAndCalculatePositionalAttributes();

          acquireAllSensorReadings(); // compass, IMU, Depth, Temp, Humidity, Pressure
  
          refreshConsoleScreen();
  
          checkForButtonPresses();
  
          performUplinkTasks();
        }
      }
      else
      {
        // get location invalid if there is no new fix to read before 1 second is up.
        if (power_up_no_fix_byte_loop_count > -1)
        {
          // Bytes are being received but no valid location fix has been seen since startup
          // Increment byte count shown until first fix received.
          M5.Lcd.setCursor(50, 90);
          M5.Lcd.printf("%d", power_up_no_fix_byte_loop_count++);
        }
      }
    }
    else
    {
      // no byte received.
    }
  }
  
  return result;
}

void refreshAndCalculatePositionalAttributes()
{
  Lat = gps.location.lat();
  lat_str = String(Lat , 7);
  Lng = gps.location.lng();
  lng_str = String(Lng , 7);
  satellites = gps.satellites.value();

  if (enableRecentCourseCalculation)
  {
    if (millis() - last_journey_commit_time > journey_calc_period)
    {
      double distanceTravelled = gps.distanceBetween(Lat, Lng, journey_lat, journey_lng);

      if (distanceTravelled > journey_min_dist)
      {
        // Must have travelled min distance and min period elapsed since last waypoint.
        // store the course travelled since last waypoint.
        journey_course = gps.courseTo(journey_lat, journey_lng, Lat, Lng);
        if (journey_course == 360)
          journey_course = 0;
        journey_distance = distanceTravelled;

        if (journey_course >= 359.5) journey_course = 0;

        if (journey_distance > 50) journey_distance = 0;    // correct for initial sample

        last_journey_commit_time = millis();
        journey_lat = Lat;
        journey_lng = Lng;
        activity_count = (activity_count + 1) % 4;
      }
    }
  }
  else
  {
    journey_course = 1;
    journey_distance = 1.1;
  }

  if  (millis() - journey_clear_period > last_journey_commit_time || journey_distance == 0)
    blackout_journey_no_movement = true;
  else
    blackout_journey_no_movement = false;

  if (directionMetric == COMPASS_HEADING)
    blackout_journey_no_movement = false;

  if (enableNavigationTargeting)
  {
    heading_to_target = gps.courseTo(Lat, Lng, nextTarget->_lat, nextTarget->_long);
    distance_to_target = gps.distanceBetween(Lat, Lng, nextTarget->_lat, nextTarget->_long);
  }
  else
  {
    heading_to_target = 1.0;
    distance_to_target = 1.1;
  }

  if (distance_to_target > 10000000.0)  // fake data received from float for No GPS.
  {
    // NO GPS Fake data from M5 53
    if (GPS_status != GPS_NO_GPS_LIVE_IN_FLOAT)
    {
      GPS_status = GPS_NO_GPS_LIVE_IN_FLOAT;
      M5.Lcd.fillScreen(TFT_BLACK);
    }
  }
  else if (distance_to_target > 7000000.0) // fake data received from float for No fix yet.
  {
    // NO Fix Fake data from M5 53
    if (GPS_status != GPS_NO_FIX_FROM_FLOAT)
    {
      GPS_status = GPS_NO_FIX_FROM_FLOAT;
      M5.Lcd.fillScreen(TFT_BLACK);
    }
  }
  else
  {
    if (GPS_status != GPS_FIX_FROM_FLOAT)
    {
      GPS_status = GPS_FIX_FROM_FLOAT;
      M5.Lcd.fillScreen(TFT_BLACK);
    }
  }
}

void acquireAllSensorReadings()
{        
  // Don't use smooth, show sample directly every 250 ms
  //        getSmoothedMagHeading(magnetic_heading);
  if (millis() > s_lastCompassDisplayRefresh + s_compassHeadingUpdateRate)
  {
    s_lastCompassDisplayRefresh = millis();

    if (compassAvailable)
    {
      if (enableTiltCompensation)
      {
        getMagHeadingTiltCompensated(magnetic_heading);
      }
      else
      {
        getMagHeadingNotTiltCompensated(magnetic_heading);
      }
    }
    else
    {
      magnetic_heading = 0;
    }

    getM5ImuSensorData(&imu_gyro_vector.x, &imu_gyro_vector.y, &imu_gyro_vector.z,
                       &imu_lin_acc_vector.x, &imu_lin_acc_vector.y, &imu_lin_acc_vector.z,
                       &imu_rot_acc_vector.x, &imu_rot_acc_vector.y, &imu_rot_acc_vector.z,
                       &imu_temperature);
  }

  if (millis() > s_lastTempHumidityDisplayRefresh + s_tempHumidityUpdateRate)
  {
    s_lastTempHumidityDisplayRefresh = millis();
    //           getTempAndHumidity(humidity, temperature);
    getTempAndHumidityAndAirPressureBME280(humidity, temperature, air_pressure, pressure_altitude);
    getDepth(depth, water_temperature, water_pressure, depth_altitude);
  }
}

const uint32_t buttonPressDurationToChangeScreen = 50;

void checkForButtonPresses()
{
  updateButtonsAndBuzzer();
  
  if (display_to_show == LOCATION_DISPLAY)
  {
    if (p_primaryButton->wasReleasefor(1000)) // Location Display: toggle ota only
    {
      toggleOTAActive();
    }
    else if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))
    {
      switchToNextDisplayToShow();
    }

    if (p_secondButton->wasReleasefor(1000)) // Location Display: toggle wifi only
    {
      toggleWiFiActive();
    }
  }
  else if (display_to_show == JOURNEY_DISPLAY)
  {
    if (p_primaryButton->wasReleasefor(1000)) // Journey Course Display: toggle send uplink messages
    {
      toggleUplinkMessageProcessAndSend();
    }
    else if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))  // change display screen
    {
      switchToNextDisplayToShow();
    }

    if (p_secondButton->wasReleasefor(1000)) // Journey Course Display: toggle uptime
    {
      toggleUptimeGlobalDisplay();
    }
  }
  else if (display_to_show == AUDIO_TEST_DISPLAY)
  {
    /*
        M5.Lcd.println("Toggle ESPNow: Top 10s\n");
        M5.Lcd.println("Start/Stop Play: Side 0.5s\n");
        M5.Lcd.println("Vol cycle: Side 2s\n");
        M5.Lcd.println("Next Track: Side 5s\n");
    */

    if (p_primaryButton->wasReleasefor(10000))    // toggle between espnow and wifi
    {
      toggleESPNowActive();
    }
    else if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))  // change display screen
    {
      switchToNextDisplayToShow();
    }

    if (p_secondButton->wasReleasefor(5000)) // Skip to next track
    {
      publishToSilkySkipToNextTrack();
    }
    else if (p_secondButton->wasReleasefor(2000)) // cycle volume up and then low at max
    {
      publishToSilkyCycleVolumeUp();
    }
    else if (p_secondButton->wasReleasefor(500)) // start/stop play
    {
      publishToSilkyTogglePlayback();
    }
  }
  else
  {
    if (p_primaryButton->wasReleasefor(10000))    // Nav Screens : emergency tweet location
    {
      showLatLongStartTime = millis();
      display_to_show = SHOW_LAT_LONG_DISPLAY;
      setTweetLocationNowFlag = true;
      setTweetEmergencyNowFlag = true;
      M5.Lcd.fillScreen(TFT_BLACK);
    }
    else if (p_primaryButton->wasReleasefor(2000)) // Nav Screens : show lat long for 5 seconds
    {
      showLatLongStartTime = millis();
      display_to_show = SHOW_LAT_LONG_DISPLAY;
      setTweetLocationNowFlag = true;
      M5.Lcd.fillScreen(TFT_BLACK);
    }
    else if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))  // change display screen
    {
      switchToNextDisplayToShow();
    }

    if (p_secondButton->wasReleasefor(10000))     // Nav Screens: force reboot on
    {
      // force reboot
      esp_restart();
    }
    else if (p_secondButton->wasReleasefor(3000))     // Nav Screens: goto last dive target (jettie)
    {
      // goto buttie dive exit (the last target on the list)
      nextTarget = diveOneTargets + 6;
      flashNextTargetOnScreen(false);
    }
    else if (p_secondButton->wasReleasefor(1000))     // Nav Screens: switch to next target
    {
      // head to next target, if at end of target list go to the top of the list
      if (++nextTarget == diveOneTargets + targetCount)
        nextTarget = diveOneTargets;
      flashNextTargetOnScreen(false);
    }
    else if (p_secondButton->wasReleasefor(250))      // Nav Screens: remind diver of current target
    {
      // don't change target - remind diver of current target
      flashNextTargetOnScreen(true);
    }
  }
}

void refreshConsoleScreen()
{

  if (display_to_show == NAV_COURSE_DISPLAY ||
      display_to_show == NAV_COMPASS_DISPLAY)
  {
    directionMetric = (display_to_show == NAV_COURSE_DISPLAY ? JOURNEY_COURSE : COMPASS_HEADING);

    // Target Course and distance is shown in Green
    // Journey course and distance over last 10 seconds shown in Red
    // HDOP quality shown in top right corner as square block. Blue best at <=1.
    // Sat count shown underneath HDOP. Red < 4, Orange < 6, Yellow < 10, Blue 10+

    M5.Lcd.setRotation(0);
    M5.Lcd.setTextSize(5);
    M5.Lcd.setCursor(0, 0);

    uint16_t x = 0, y = 0, degree_offset = 0, cardinal_offset = 0, hdop = 0, metre_offset = 0;

    if (GPS_status == GPS_NO_GPS_LIVE_IN_FLOAT)
    {
      M5.Lcd.setCursor(5, 0);
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);

      M5.Lcd.print(" NO\n");
      M5.Lcd.setCursor(21, 48);
      M5.Lcd.print("GPS");

      M5.Lcd.setTextSize(2);
    }
    else if (GPS_status == GPS_NO_FIX_FROM_FLOAT)
    {
      M5.Lcd.setCursor(5, 0);
      M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);

      M5.Lcd.print(" NO\n");
      M5.Lcd.setCursor(21, 48);
      M5.Lcd.print("FIX");

      M5.Lcd.setTextSize(2);
    }
    else if (GPS_status == GPS_FIX_FROM_FLOAT)
    {
      M5.Lcd.setCursor(5, 0);
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

      // Display Green heading to target at top with degrees sign suffix
      M5.Lcd.printf("%3.0f", heading_to_target);
      M5.Lcd.setTextSize(2);
      degree_offset = -2;
      x = M5.Lcd.getCursorX();
      y = M5.Lcd.getCursorY();
      M5.Lcd.setCursor(x, y + degree_offset);
      M5.Lcd.print("o ");

      // Display Cardinal underneath degrees sign
      cardinal_offset = 21;
      M5.Lcd.setCursor(x, y + cardinal_offset);
      M5.Lcd.printf("%s ", getCardinal(heading_to_target).c_str());

      // Display HDOP signal quality as small coloured dot
      hdop = gps.hdop.hdop();
      if (hdop > 10)
        M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
      else if (hdop > 5)
        M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
      else if (hdop > 1)
        M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
      else
        M5.Lcd.setTextColor(TFT_BLUE, TFT_BLACK);

      M5.Lcd.setTextSize(5);
      M5.Lcd.setCursor(x + 10, y - 25);
      M5.Lcd.print(".");

      // Display number of satellites
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(x + 8, y + 40);
      if (satellites < 4)
        M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
      else if (hdop < 6)
        M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
      else if (hdop < 10)
        M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
      else
        M5.Lcd.setTextColor(TFT_BLUE, TFT_BLACK);
      M5.Lcd.printf("%2lu", satellites);

      // Display distance to target in metres, with by 'm' suffix
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
      M5.Lcd.setCursor(x, y);
      M5.Lcd.setTextSize(5);

      if (distance_to_target < 1000)      //     (less than 1km)
      {
        M5.Lcd.printf("\n%3.0f", distance_to_target);
        M5.Lcd.setTextSize(3);

        x = M5.Lcd.getCursorX();
        y = M5.Lcd.getCursorY();

        metre_offset = 14;
        M5.Lcd.setCursor(x, y + metre_offset);
        M5.Lcd.print("m");

        // Clear any extra line used by distance where distance > 999m and wrap has occurred
        M5.Lcd.setTextSize(0);
        M5.Lcd.print("\n\n\n\n\n");
      }
      else
      {
        M5.Lcd.printf("\n*%3d", ((uint32_t)distance_to_target) % 1000);
      }

    }

    float directionOfTravel = (directionMetric == JOURNEY_COURSE ? journey_course : magnetic_heading);

    // Display Journey Course with degrees suffix
    // this is the direction travelled in last x seconds
    // Black out the Journey Course if no recent movement
    M5.Lcd.setTextSize(5);

    if (directionMetric == JOURNEY_COURSE)
    {
      M5.Lcd.setTextColor((blackout_journey_no_movement ? TFT_BLACK : TFT_RED), TFT_BLACK);
    }
    else
    {
      // never black out mag compass heading
      M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    }

    M5.Lcd.printf("%3.0f", directionOfTravel);

    x = M5.Lcd.getCursorX();
    y = M5.Lcd.getCursorY();

    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(x, y + degree_offset);

    if (directionMetric == JOURNEY_COURSE && GPS_status == GPS_FIX_FROM_FLOAT)
    {
      // Display small rotating line character to indicate a new journey datapoint has been recorded
      M5.Lcd.printf("o %c", activity_indicator[activity_count]);

      // Display Cardinal underneath degrees sign
      cardinal_offset = 21;
      M5.Lcd.setCursor(x, y + cardinal_offset);
      M5.Lcd.printf("%s ", getCardinal(directionOfTravel).c_str());

      // Display distance travelled during last journey course measurement with 'm' suffix
      M5.Lcd.setCursor(x, y);
      M5.Lcd.setTextSize(5);
      M5.Lcd.printf("\n%3.0f", journey_distance);

      M5.Lcd.setTextSize(3);

      x = M5.Lcd.getCursorX();
      y = M5.Lcd.getCursorY();

      M5.Lcd.setCursor(x, y + metre_offset);
      M5.Lcd.print("m");
      M5.Lcd.setTextSize(5);
      refreshDirectionGraphic(directionOfTravel, heading_to_target);
      refreshDepthDisplay();
    }
    else if (directionMetric == COMPASS_HEADING)
    {
      // Display degrees
      M5.Lcd.printf("o ");

      // Display Cardinal underneath degrees sign
      const uint16_t cardinal_offset = 21;
      M5.Lcd.setCursor(x, y + cardinal_offset);
      M5.Lcd.printf("%s ", getCardinal(directionOfTravel).c_str());

      // Display temp and humidity
      M5.Lcd.setCursor(x, y);
      M5.Lcd.setTextSize(2);
      M5.Lcd.printf("\n\n\n%2.1f", temperature);

      x = M5.Lcd.getCursorX();
      y = M5.Lcd.getCursorY();

      const uint16_t temp_degrees_offset = -2;
      M5.Lcd.setCursor(x + 3, y + temp_degrees_offset);
      M5.Lcd.setTextSize(0);
      M5.Lcd.printf("o ", temperature);
      M5.Lcd.setTextSize(2);
      M5.Lcd.printf("C %3.0f%%", humidity);

      if (GPS_status == GPS_FIX_FROM_FLOAT)
        refreshDirectionGraphic(directionOfTravel, heading_to_target);

      refreshDepthDisplay();

      blackout_journey_no_movement = false;
    }
    else
    {

    }
  }
  else if (display_to_show == AUDIO_TEST_DISPLAY)
  {
    M5.Lcd.setRotation(1);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 0);

    M5.Lcd.setTextColor(TFT_BLACK, TFT_GREEN);
    M5.Lcd.println("Toggle ESPNow: Top 10s\n");
    M5.Lcd.println("Start/Stop Play: Side 0.5s\n");
    M5.Lcd.println("Vol cycle: Side 2s\n");
    M5.Lcd.println("Next Track: Side 5s\n");
  }
  else if (display_to_show == SHOW_LAT_LONG_DISPLAY)
  {
    M5.Lcd.setRotation(1);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(0, 0);
    /*
            if (setTweetEmergencyNowFlag == true)
            {
              M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
              M5.Lcd.printf("SOS TWEET SENT\n", Lat);
              sleep(3000);
            }
            else
            {
              M5.Lcd.setTextColor(TFT_BLUE, TFT_YELLOW);
              M5.Lcd.printf(" TWEET SENT! \n", Lat);
            }
    */

    M5.Lcd.setTextColor(TFT_BLACK, TFT_GREEN);
    M5.Lcd.printf("Location Here\n");
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.printf("La:%.5f\n", Lat);
    M5.Lcd.printf("Lo:%.5f\n", Lng);
    M5.Lcd.printf("%02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    M5.Lcd.printf("%02d/%02d/%02d\n", gps.date.day(), gps.date.month(), gps.date.year());

    if (millis() > showLatLongStartTime + showLatLongHoldDuration)
    {
      showLatLongStartTime = 0;
      display_to_show = NAV_COMPASS_DISPLAY;
    }
  }
  else if (display_to_show == LOCATION_DISPLAY)
  {
    M5.Lcd.setRotation(1);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

    M5.Lcd.setCursor(5, 0);
    M5.Lcd.printf("La:%.5f   ", Lat);
    M5.Lcd.setCursor(5, 17);
    M5.Lcd.printf("Lo:%.5f   ", Lng);
    M5.Lcd.setCursor(5, 34);

    M5.Lcd.printf("Depth:%.0f m  ", depth);
    M5.Lcd.setCursor(5, 51);
    M5.Lcd.printf("Water P:%.1f% Bar", water_pressure);

    M5.Lcd.setCursor(5, 68);
    M5.Lcd.printf("T:%s", nextTarget->_label);

    M5.Lcd.setCursor(5, 85);
    if (WiFi.status() == WL_CONNECTED)
      M5.Lcd.printf("IP: %s", WiFi.localIP().toString());
    else
      M5.Lcd.printf("IP: No WiFi");

    M5.Lcd.setCursor(5, 102);
    M5.Lcd.printf("crs: %.0f d: %.0f    ", heading_to_target, distance_to_target);
  }
  else if (display_to_show == JOURNEY_DISPLAY)
  {
    M5.Lcd.setRotation(1);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

    M5.Lcd.setCursor(5, 0);
    M5.Lcd.printf("V:%.2fV I:%.0fmA  ", M5.Axp.GetVBusVoltage(), M5.Axp.GetVBusCurrent());
    M5.Lcd.setCursor(5, 17);


    M5.Lcd.printf("OTA:%hu Uplink:%hu", otaActiveListening,  enableUplinkComms);

    M5.Lcd.setCursor(5, 34);
    M5.Lcd.printf("Wifi:%hu %s", WiFi.status() == WL_CONNECTED, ssid_connected);
    M5.Lcd.setCursor(5, 51);
    M5.Lcd.printf("Fix:%lu Up:%lu", fixCount, uplinkMessageCount);
    M5.Lcd.setCursor(5, 68);
    M5.Lcd.printf("chk+:%lu chk-:%lu", newPassedChecksum, newFailedChecksum);
    M5.Lcd.setCursor(5, 85);
    M5.Lcd.printf("tc:%.0f d:%.0f  ", heading_to_target, distance_to_target);
    M5.Lcd.setCursor(5, 102);
    M5.Lcd.printf("%02d:%02d:%02d UTC", gps.time.hour(), gps.time.minute(), gps.time.second());
    M5.Lcd.setCursor(5, 119);
    M5.Lcd.printf("Uptime:%.1f", ((float)millis() / 1000.0));
  }
  else
  {
    M5.Lcd.setCursor(5, 17);
    M5.Lcd.printf("NULL DISPLAY");
  }

  // overlay count up / power on time in seconds.
  if (enableGlobalUptimeDisplay)
  {
    M5.Lcd.setCursor(0, SCREEN_WIDTH - 15);
    M5.Lcd.setRotation(1);
    M5.Lcd.setTextFont(1);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
    M5.Lcd.printf(" Uptime: %.1f ", ((float)millis() / 1000.0));
  }
}


void performUplinkTasks()
{
  if (enableUplinkComms)
  {
    if (uplinkMessageCount % telemetry_send_full_message_duty_cycle == 0)
    {
      fp_sendUplinkMessage();
    }
    else
    {
      sendBasicWithDepthUplinkTelemetryMessage();
      //            sendBasicUplinkTelemetryMessage();
    }

    uplinkMessageCount++;
  }    
}

void refreshGlobalStatusDisplay()
{
  if (power_up_no_fix_byte_loop_count > 0)
  {
    // Bytes have been received from the Float UART/Serial but no fix has been received since power on.
    // The 'No Fix' only shown on first acquisition.
    M5.Lcd.setCursor(55, 5);
    M5.Lcd.setTextSize(4);
    M5.Lcd.printf("No Fix\n");
    M5.Lcd.setCursor(110, 45);
    M5.Lcd.printf("%c", activity_indicator[(++activity_count) % 4]);
    delay(250); // no fix wait
  }
  else if (power_up_no_fix_byte_loop_count != -1)
  {
    // No GPS is reported when no bytes have ever been received on the Float UART/Serial.
    // Once messages start being received, this is blocked as it is normal
    // to have gaps in the stream. There is no indication if GPS stream hangs
    // after first byte received, eg no bytes within 10 seconds.
    M5.Lcd.setCursor(55, 5);
    M5.Lcd.setTextSize(4);
    M5.Lcd.printf("No GPS\n");
    M5.Lcd.setCursor(110, 45);
    M5.Lcd.printf("%c", activity_indicator[(++activity_count) % 4]);
    delay(250); // no fix wait
  }
  else
  {
    // do nothing - the two conditions above are dealing with initial conditions before any fix/msg received from float serial
  }
}

void sendNoUplinkTelemetryMessages()
{
  /// do nothing
}

void sendBasicUplinkTelemetryMessage()
{
  sendUplinkTelemetryMessageV1();
}

void sendBasicWithDepthUplinkTelemetryMessage()
{
  sendUplinkTelemetryMessageV2();
}

void sendFullUplinkTelemetryMessage()
{
  sendUplinkTelemetryMessageV5();
}

void sendUplinkTelemetryMessageV1()
{
  // if 5ms after fix received then send the uplink msg, only get here if no bytes received from Float Serial.
  const uint32_t quietTimeMsBeforeUplink = 5;
  if (millis() > latestFixTimeStamp + quietTimeMsBeforeUplink)
  {
    latestFixTimeStamp = CLEARED_FIX_TIME_STAMP;

    // this is 20 bytes, 160 bits
    // format: uint16_t len, uint16_t msgtype, unit16_t depth (*10),uint16_t heading (*10),uint16_t temp (*10),uint16_t humid (*10), uint16_t pressure (*10), uint16_t checksum

    // fixed format
    uint16_t uplink_length = 22;   // including length and checksum.
    uint16_t uplink_msgtype = 0;   // 0 is telemetry message zero!
    uint16_t uplink_depth = depth;
    uint16_t uplink_water_pressure = water_pressure * 100.0;
    uint16_t uplink_water_temperature = water_temperature * 10.0;
    uint16_t uplink_enclosure_temperature = temperature * 10.0;
    uint16_t uplink_enclosure_humidity = humidity * 10.0;
    uint16_t uplink_enclosure_air_pressure = air_pressure * 10.0;
    uint16_t uplink_heading = magnetic_heading * 10.0;
    uint16_t uplink_flags = setTweetLocationNowFlag | (setTweetEmergencyNowFlag << 1);
    uint16_t uplink_checksum = uplink_length ^ uplink_msgtype ^ uplink_depth ^ uplink_water_pressure ^ uplink_water_temperature ^
                               uplink_enclosure_temperature ^ uplink_enclosure_humidity ^ uplink_enclosure_air_pressure ^ uplink_flags ^ uplink_heading;

    telemetryMessage[0] = uplink_length;
    telemetryMessage[1] = uplink_msgtype;
    telemetryMessage[2] = uplink_depth;
    telemetryMessage[3] = uplink_water_pressure;
    telemetryMessage[4] = uplink_water_temperature;
    telemetryMessage[5] = uplink_enclosure_temperature;
    telemetryMessage[6] = uplink_enclosure_humidity;
    telemetryMessage[7] = uplink_enclosure_air_pressure;
    telemetryMessage[8] = uplink_heading;
    telemetryMessage[9] = uplink_flags;
    telemetryMessage[10] = uplink_checksum;

    float_serial.write(uplink_preamble_pattern);

    float_serial.write((char*)telemetryMessage, uplink_length);

    // clear flags
    if (setTweetLocationNowFlag == true)
      setTweetLocationNowFlag = false;

    if (setTweetEmergencyNowFlag == true)
      setTweetEmergencyNowFlag = false;
  }
}

void sendUplinkTelemetryMessageV2()
{
  // if 2ms after fix received then send the uplink msg, only get here if no bytes received from Float Serial.
  const uint32_t quietTimeMsBeforeUplink = 5;
  if (millis() > latestFixTimeStamp + quietTimeMsBeforeUplink)
  {
    latestFixTimeStamp = CLEARED_FIX_TIME_STAMP;

    // this is 42 bytes, 672 bits
    // format: uint16_t len, uint16_t msgtype, unit16_t depth (*10),uint16_t heading (*10),uint16_t temp (*10),uint16_t humid (*10), uint16_t pressure (*10), uint16_t checksum

    // fixed format

    uint16_t uplink_length = 46;   // including length and checksum.
    uint16_t uplink_msgtype = 0;   // 0 is full fat telemetry message zero!
    uint16_t uplink_depth = depth * 10.0;
    uint16_t uplink_water_pressure = water_pressure * 100.0;
    uint16_t uplink_water_temperature = water_temperature * 10.0;
    uint16_t uplink_enclosure_temperature = temperature * 10.0;
    uint16_t uplink_enclosure_humidity = humidity * 10.0;
    uint16_t uplink_enclosure_air_pressure = air_pressure * 10.0;
    uint16_t uplink_heading = magnetic_heading * 10.0;

    uint16_t uplink_heading_to_target = heading_to_target * 10.0;
    uint16_t uplink_distance_to_target = (distance_to_target < 6499 ? distance_to_target * 10.0 : 64999);
    uint16_t uplink_journey_course = journey_course * 10.0;
    uint16_t uplink_journey_distance = journey_distance * 100.0;

    uint16_t uplink_mako_screen_display = 0xFFFF;     // TO DO
    uint16_t uplink_mako_seconds_on = millis() / 1000.0;
    uint16_t uplink_mako_user_action = 0xFFFF;    // has mode been changed or action done by user?
    uint16_t uplink_mako_AXP192_temp = M5.Axp.GetTempInAXP192() * 10.0;
    uint16_t uplink_mako_usb_voltage = M5.Axp.GetVBusVoltage() * 1000.0;
    uint16_t uplink_mako_usb_current = M5.Axp.GetVBusCurrent() * 100.0;

    uint16_t uplink_mako_bat_voltage = M5.Axp.GetBatVoltage() * 1000.0;
    uint16_t uplink_mako_bat_charge_current = M5.Axp.GetBatChargeCurrent() * 100.0;


    uint16_t uplink_flags = setTweetLocationNowFlag | (setTweetEmergencyNowFlag << 1);
    uint16_t uplink_checksum = uplink_length ^ uplink_msgtype ^ uplink_depth ^ uplink_water_pressure ^ uplink_water_temperature ^
                               uplink_enclosure_temperature ^ uplink_enclosure_humidity ^ uplink_enclosure_air_pressure ^ uplink_flags ^ uplink_heading;

    uint8_t number_uplink_metrics = 22;
    uint16_t* nextMetric = telemetryMessage;

    *(nextMetric++) = uplink_length;
    *(nextMetric++) = uplink_msgtype;
    *(nextMetric++) = uplink_depth;
    *(nextMetric++) = uplink_water_pressure;
    *(nextMetric++) = uplink_water_temperature;
    *(nextMetric++) = uplink_enclosure_temperature;
    *(nextMetric++) = uplink_enclosure_humidity;
    *(nextMetric++) = uplink_enclosure_air_pressure;
    *(nextMetric++) = uplink_heading;
    *(nextMetric++) = uplink_heading_to_target;
    *(nextMetric++) = uplink_distance_to_target;
    *(nextMetric++) = uplink_journey_course;
    *(nextMetric++) = uplink_journey_distance;
    *(nextMetric++) = uplink_mako_screen_display;
    *(nextMetric++) = uplink_mako_seconds_on;
    *(nextMetric++) = uplink_mako_user_action;
    *(nextMetric++) = uplink_mako_AXP192_temp;
    *(nextMetric++) = uplink_mako_usb_voltage;
    *(nextMetric++) = uplink_mako_usb_current;
    *(nextMetric++) = uplink_mako_bat_voltage;
    *(nextMetric++) = uplink_mako_bat_charge_current;
    *(nextMetric++) = uplink_flags;
    *(nextMetric++) = 0;   // initialise checksum to zero prior to computing checksum

    // calculate and store checksum by xor'ing all words.
    for (int i = 0; i < number_uplink_metrics; i++)
    {
      telemetryMessage[number_uplink_metrics] ^= telemetryMessage[i];
    }

    float_serial.write(uplink_preamble_pattern);

    float_serial.write((char*)telemetryMessage, uplink_length);

    // clear flags
    if (setTweetLocationNowFlag == true)
      setTweetLocationNowFlag = false;

    if (setTweetEmergencyNowFlag == true)
      setTweetEmergencyNowFlag = false;
  }
}


void sendUplinkTelemetryMessageV5()
{
  // if 1ms after fix received then send the uplink msg, only get here if no bytes received from Float Serial.
  const uint32_t quietTimeMsBeforeUplink = 5;
  if (millis() > latestFixTimeStamp + quietTimeMsBeforeUplink)
  {
    latestFixTimeStamp = CLEARED_FIX_TIME_STAMP;

    // this is 57 words, 114 bytes including checksum (56 metrics)
    // format: uint16_t len, uint16_t msgtype, unit16_t depth (*10),uint16_t heading (*10),uint16_t temp (*10),uint16_t humid (*10), uint16_t pressure (*10), uint16_t checksum

    // fixed format

    uint16_t uplink_length = 116;   // bytes to transmit in this message - including length and checksum. 55 x 2 byte words == 110 bytes
    uint8_t  number_uplink_metrics = 57; // number of metrics including length, not including checksum
    uint16_t uplink_msgtype = 0;   // 0 is full fat telemetry message zero!
    uint16_t uplink_depth = depth * 10.0;
    uint16_t uplink_water_pressure = water_pressure * 100.0;

    uint16_t uplink_water_temperature = water_temperature * 10.0;
    uint16_t uplink_enclosure_temperature = temperature * 10.0;
    uint16_t uplink_enclosure_humidity = humidity * 10.0;
    uint16_t uplink_enclosure_air_pressure = air_pressure * 10.0;

    uint16_t uplink_heading = magnetic_heading * 10.0;

    uint16_t uplink_heading_to_target = heading_to_target * 10.0;
    uint16_t uplink_distance_to_target = (distance_to_target < 6499 ? distance_to_target * 10.0 : 64999);
    uint16_t uplink_journey_course = journey_course * 10.0;
    uint16_t uplink_journey_distance = journey_distance * 100.0;
    
    uint16_t uplink_mako_seconds_on = millis() / 1000.0 / 60.0;
    uint16_t uplink_mako_user_action = 0xFFFF;    // has mode been changed or action done by user?
    uint16_t uplink_mako_AXP192_temp = (M5.Axp.GetTempData() * 0.1 - 144.7) * 10.0; // ?????

    uint16_t uplink_mako_usb_voltage = M5.Axp.GetVBusVoltage() * 1000.0;
    uint16_t uplink_mako_usb_current = M5.Axp.GetVBusCurrent() * 100.0;

    uint16_t uplink_mako_bat_voltage = (M5.Axp.GetVbatData() * 1.1 / 1000) * 1000.0;
    uint16_t uplink_mako_bat_charge_current = (M5.Axp.GetIchargeData() / 2) * 100.0;

    float uplink_mako_lsm_mag_x = magnetometer_vector.x;
    float uplink_mako_lsm_mag_y = magnetometer_vector.y;
    float uplink_mako_lsm_mag_z = magnetometer_vector.z;
    float uplink_mako_lsm_acc_x = accelerometer_vector.x;
    float uplink_mako_lsm_acc_y = accelerometer_vector.y;
    float uplink_mako_lsm_acc_z = accelerometer_vector.z;

    float uplink_mako_imu_gyro_x = imu_gyro_vector.x;
    float uplink_mako_imu_gyro_y = imu_gyro_vector.y;
    float uplink_mako_imu_gyro_z = imu_gyro_vector.z;

    float uplink_mako_imu_lin_acc_x = imu_lin_acc_vector.x;
    float uplink_mako_imu_lin_acc_y = imu_lin_acc_vector.y;
    float uplink_mako_imu_lin_acc_z = imu_lin_acc_vector.z;

    float uplink_mako_imu_rot_acc_x = imu_rot_acc_vector.x;
    float uplink_mako_imu_rot_acc_y = imu_rot_acc_vector.y;
    float uplink_mako_imu_rot_acc_z = imu_rot_acc_vector.z;

    float uplink_mako_imu_temperature = imu_temperature * 10.0;

    uint16_t uplink_flags = setTweetLocationNowFlag | (setTweetEmergencyNowFlag << 1);

    uint16_t* nextMetric = telemetryMessage;

    // 21x16 bit words (21 x 2 byte metrics)
    *(nextMetric++) = uplink_length;
    *(nextMetric++) = uplink_msgtype;
    *(nextMetric++) = uplink_depth;
    *(nextMetric++) = uplink_water_pressure;
    *(nextMetric++) = uplink_water_temperature;
    *(nextMetric++) = uplink_enclosure_temperature;
    *(nextMetric++) = uplink_enclosure_humidity;
    *(nextMetric++) = uplink_enclosure_air_pressure;
    *(nextMetric++) = uplink_heading;
    *(nextMetric++) = uplink_heading_to_target;
    *(nextMetric++) = uplink_distance_to_target;
    *(nextMetric++) = uplink_journey_course;
    *(nextMetric++) = uplink_journey_distance;
    
    switch (display_to_show)
    {
      case NAV_COMPASS_DISPLAY: displayLabel[0] = navCompassDisplayLabel[0]; displayLabel[1] = navCompassDisplayLabel[1]; break;
      case NAV_COURSE_DISPLAY:  displayLabel[0] = navCourseDisplayLabel[0]; displayLabel[1] = navCourseDisplayLabel[1]; break;
      case LOCATION_DISPLAY:    displayLabel[0] = locationDisplayLabel[0]; displayLabel[1] = locationDisplayLabel[1]; break;
      case JOURNEY_DISPLAY:     displayLabel[0] = journeyDisplayLabel[0]; displayLabel[1] = journeyDisplayLabel[1]; break;
      case SHOW_LAT_LONG_DISPLAY: displayLabel[0] = showLatLongDisplayLabel[0]; displayLabel[1] = showLatLongDisplayLabel[1]; break;
      case AUDIO_TEST_DISPLAY:  displayLabel[0] = audioTestDisplayLabel[0]; displayLabel[1] = audioTestDisplayLabel[1]; break;
      default:                  displayLabel[0] = undefinedDisplayLabel[0]; displayLabel[1] = undefinedDisplayLabel[1]; break;
    }

    *(nextMetric++) = (uint16_t)(displayLabel[0]) + (((uint16_t)(displayLabel[1])) << 8); // which display is shown on the console

    *(nextMetric++) = uplink_mako_seconds_on;
    *(nextMetric++) = uplink_mako_user_action;
    *(nextMetric++) = uplink_mako_AXP192_temp;
    *(nextMetric++) = uplink_mako_usb_voltage;
    *(nextMetric++) = uplink_mako_usb_current;
    *(nextMetric++) = uplink_mako_bat_voltage;
    *(nextMetric++) = uplink_mako_bat_charge_current;

    char* p = NULL;

    // 6x32 bit words (floats) - (12 x 2 byte metrics)
    p = (char*) &uplink_mako_lsm_mag_x;         *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_lsm_mag_y;         *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_lsm_mag_z;         *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_lsm_acc_x;         *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_lsm_acc_y;         *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_lsm_acc_z;         *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);

    // 9x32 bit words (floats) - (18 x 2 byte metrics)
    p = (char*) &uplink_mako_imu_gyro_x;        *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_gyro_y;        *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_gyro_z;        *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_lin_acc_x;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_lin_acc_y;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_lin_acc_z;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_rot_acc_x;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_rot_acc_y;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_rot_acc_z;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);

    // 2 x 16 bit words (2 x 2 byte metrics)
    *(nextMetric++) = uplink_mako_imu_temperature;
    *(nextMetric++) = newWayMarker; // guidance graphic enum

    switch (newWayMarker)         // guidance label
    {
      case BLACKOUT_MARKER:  newWayMarkerLabel[0] = shortBlackOut[0]; newWayMarkerLabel[1] = shortBlackOut[1]; break;
      case GO_ANTICLOCKWISE_MARKER:  newWayMarkerLabel[0] = shortAntiClockwise[0]; newWayMarkerLabel[1] = shortAntiClockwise[1]; break;
      case GO_AHEAD_MARKER:  newWayMarkerLabel[0] = shortAhead[0]; newWayMarkerLabel[1] = shortAhead[1]; break;
      case GO_CLOCKWISE_MARKER:  newWayMarkerLabel[0] = shortClockwise[0]; newWayMarkerLabel[1] = shortClockwise[1]; break;
      case GO_TURN_AROUND_MARKER:  newWayMarkerLabel[0] = shortTurnAround[0]; newWayMarkerLabel[1] = shortTurnAround[1]; break;
      case UNKNOWN_MARKER:  newWayMarkerLabel[0] = shortUnknownMarker[0]; newWayMarkerLabel[1] = shortUnknownMarker[1]; break;
      default: newWayMarkerLabel[0] = shortUndefinedMarker[0]; newWayMarkerLabel[1] = shortUndefinedMarker[1]; break; // undefined
    }

    switch (directionMetric)
    {
      case JOURNEY_COURSE: directionMetricLabel[0] = shortJourneyCourseDirectionMetric[0]; directionMetricLabel[1] = shortJourneyCourseDirectionMetric[1]; break;
      case COMPASS_HEADING: directionMetricLabel[0] = shortCompassHeadingDirectionMetric[0]; directionMetricLabel[1] = shortCompassHeadingDirectionMetric[1]; break;
      default: directionMetricLabel[0] = shortUndefinedDirectionMetric[0]; directionMetricLabel[1] = shortUndefinedDirectionMetric[1]; break;
    }

    // 2 x 16 bit words (2 x 2 byte metrics)
    *(nextMetric++) = (uint16_t)(newWayMarkerLabel[0]) + (((uint16_t)(newWayMarkerLabel[1])) << 8); // guidance graphic
    *(nextMetric++) = (uint16_t)(directionMetricLabel[0]) + (((uint16_t)(directionMetricLabel[1])) << 8); // whether course or compass heading is shown on display

    // 2 x 16 bit words (1 x 2 byte metric plus 2 byte checksum)
    *(nextMetric++) = uplink_flags;
    *(nextMetric++) = 0;   // initialise checksum to zero prior to computing checksum

    // calculate and store checksum by xor'ing all words.
    for (int i = 0; i < number_uplink_metrics; i++)
    {
      telemetryMessage[number_uplink_metrics] ^= telemetryMessage[i];
    }

    float_serial.write(uplink_preamble_pattern);

    float_serial.write((char*)telemetryMessage, uplink_length);


    // clear flags
    if (setTweetLocationNowFlag == true)
      setTweetLocationNowFlag = false;

    if (setTweetEmergencyNowFlag == true)
      setTweetEmergencyNowFlag = false;
  }
}

void sendUplinkTestMessage()
{
  // if 5ms after fix received then send the uplink msg, only get here if no bytes received from Float Serial.
  const uint32_t quietTimeMsBeforeUplink = 5;
  if (millis() > latestFixTimeStamp + quietTimeMsBeforeUplink)
  {
    latestFixTimeStamp = CLEARED_FIX_TIME_STAMP;

    // now send one uplink msg, cycling around the 5 msgs.

    float_serial.write(uplink_preamble_pattern);

    for (int i = 0; i < 4; i++)
    {
      float_serial.write(uplinkTestMessages[nextUplinkMessage]);
    }
    nextUplinkMessage = (nextUplinkMessage + 1) % 4;
  }
}


void flipAutoPowerOffMode()
{
  M5.Lcd.fillScreen(TFT_MAGENTA);
  updateButtonsAndBuzzer();
  M5.Beep.setBeep(1400, 100);
  updateButtonsAndBuzzer();

  autoShutdownOnNoUSBPower = !autoShutdownOnNoUSBPower;

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextSize(4);
  if (autoShutdownOnNoUSBPower)
    M5.Lcd.print("  Auto\nShutdown\n   On");
  else
    M5.Lcd.print("  Auto\nShutdown\n   Off");

  delay(2000);
  M5.Beep.setBeep(1200, 100);
  updateButtonsAndBuzzer();
  M5.Lcd.fillScreen(TFT_BLACK);
}

void testForShake()
{
  const float accel_trigger = 30.0;

  if (mag_accel_x * mag_accel_x + mag_accel_y * mag_accel_y + mag_accel_z * mag_accel_z > accel_trigger * accel_trigger)
  {
    flipAutoPowerOffMode();
  }

  //    if (mag_accel_x > accel_trigger || mag_accel_y > accel_trigger || accel_trigger > accel_trigger ||
  //     mag_accel_x < -accel_trigger || mag_accel_y < -accel_trigger || -accel_trigger < -accel_trigger)
  //  {
  //    flipAutoPowerOffMode();
  //  }
}

void testForSingleButtonPressAutoShutdownChange()
{
  updateButtonsAndBuzzer();

  // not used yet
  if (p_primaryButton->wasReleasefor(750) || p_secondButton->wasReleasefor(750))
  {
    flipAutoPowerOffMode();
  }
}

void testForDualButtonPressAutoShutdownChange()
{
  updateButtonsAndBuzzer();

  // currently not called, for switching into pressure pot test mode, no cable.
  if (p_primaryButton->wasReleasefor(1000) && p_secondButton->wasReleasefor(1000))
  {
    flipAutoPowerOffMode();
  }
}

void refreshDepthDisplay()
{
  M5.Lcd.setCursor(30, 151);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextFont(0);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.printf("%4.1fm", depth);
}

void refreshDirectionGraphic( float directionOfTravel,  float headingToTarget)
{
  if (!enableNavigationGraphics)
    return;

  // Calculate whether the traveller needs to continue straight ahead,
  // rotate clockwise or rotate anticlockwise and update graphic.
  // Blacks out if no journey recorded.
  int16_t edgeBound = 15;    // If journey course within +- 15 degrees of target heading then go ahead

  int16_t normaliser = (int16_t)(directionOfTravel);

  int16_t d = (int16_t)directionOfTravel - normaliser;  // directionofTravel normalised to zero
  int16_t t = (int16_t)headingToTarget - normaliser;    // headingToTarget normalised.
  if (t > 180)                  // normalise to range -179 to +180 degrees
    t -= 360;
  else if (t <= -180)
    t += 360;

  int16_t e1 = t - edgeBound;   // left-most edge to target
  if (e1 > 180)                 // normalise to range -179 to +180 degrees
    e1 -= 360;
  else if (e1 <= -180)
    e1 += 360;

  int16_t e2 = t + edgeBound;   // right-most edge to target
  if (e2 > 180)                 // normalise to range -179 to +180 degrees
    e2 -= 360;
  else if (e2 <= -180)
    e2 += 360;

  int16_t o = t + 180;          // opposite heading to target
  if (o > 180)                  // normalise to range -179 to +180 degrees
    o -= 360;
  else if (o <= -180)
    o += 360;


  if (blackout_journey_no_movement)
  {
    goBlackout();
    lastWayMarker = BLACKOUT_MARKER;
  }
  else
  {
    if (millis() - lastWayMarkerChangeTimestamp > 1000)
    {
      lastWayMarkerChangeTimestamp = millis();

      if (e1 <= d && d <= e2)     // scenario 1
      {
        newWayMarker = GO_AHEAD_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goAhead();
          lastWayMarker = newWayMarker;
        }
      }
      else if (e1 > e2)           // scenario 4
      {
        newWayMarker = GO_TURN_AROUND_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goTurnAround();
          lastWayMarker = newWayMarker;
        }
      }
      else if (o <= d && d <= e1) // scenario 2
      {
        newWayMarker = GO_CLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goClockwise();
          lastWayMarker = newWayMarker;
        }
      }
      else if (e2 <= d && d <= o) // scenario 3
      {
        newWayMarker = GO_ANTICLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goAntiClockwise();
          lastWayMarker = newWayMarker;
        }
      }
      else if (o <= d && d <= e1) // scenario 5
      {
        newWayMarker = GO_CLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goClockwise();
          lastWayMarker = newWayMarker;
        }
      }
      else if (e2 <= d && d <= o) // scenario 6
      {
        newWayMarker = GO_ANTICLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goAntiClockwise();
          lastWayMarker = newWayMarker;
        }
      }
      else
      {
        newWayMarker = UNKNOWN_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goUnknown();
          lastWayMarker = newWayMarker;
        }
      }
    }
  }
}



void refreshDirectionGraphicOld( float directionOfTravel,  float headingToTarget)
{
  if (!enableNavigationGraphics)
    return;

  // Calculate whether the traveller needs to continue straight ahead,
  // rotate clockwise or rotate anticlockwise and update graphic.
  // Blacks out if no journey recorded.
  uint16_t edgeBound = 15;    // If journey course within +- 15 degrees of target heading then go ahead
  uint16_t edgeRight = (uint16_t)(headingToTarget + edgeBound) % 360;
  uint16_t edgeLeft = (uint16_t)(headingToTarget - edgeBound) % 360;

  uint16_t edgeOpposingTarget = (uint16_t)(headingToTarget + 180) % 360;

  if (blackout_journey_no_movement)
  {
    goBlackout();
    lastWayMarker = BLACKOUT_MARKER;
  }
  else
  {
    if (millis() - lastWayMarkerChangeTimestamp > 1000)
    {
      lastWayMarkerChangeTimestamp = millis();

      uint16_t shift = 0;

      if (edgeLeft > edgeOpposingTarget && edgeRight < edgeOpposingTarget)
        // one edge either side of the 0 discontinuity, add 180 to all headings
        shift = 180;
      else
        shift = 0;

      edgeLeft = (edgeLeft + shift) % 360;
      edgeRight = (edgeRight + shift) % 360;
      directionOfTravel = (uint16_t)(directionOfTravel + shift) % 360;
      edgeOpposingTarget = (edgeOpposingTarget + shift) % 360;
      headingToTarget = (uint16_t)(headingToTarget + shift) % 360;

      // Check for straight ahead.
      if (directionOfTravel >= edgeLeft && directionOfTravel <= edgeRight)
      {
        newWayMarker = GO_AHEAD_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goAhead();
          lastWayMarker = newWayMarker;
        }
      }
      else
      {
        if (directionOfTravel > edgeRight && directionOfTravel < edgeOpposingTarget)
        {
          newWayMarker = GO_ANTICLOCKWISE_MARKER;
          if (lastWayMarker != newWayMarker)
          {
            goAntiClockwise();
            lastWayMarker = newWayMarker;
          }
          return;
        }

        if (directionOfTravel > edgeRight && directionOfTravel >= edgeOpposingTarget)
        {
          newWayMarker = GO_CLOCKWISE_MARKER;
          if (lastWayMarker != newWayMarker)
          {
            goClockwise();
            lastWayMarker = newWayMarker;
          }
          return;
        }

        if (directionOfTravel < edgeLeft && directionOfTravel >= edgeOpposingTarget)
        {
          newWayMarker = GO_CLOCKWISE_MARKER;
          if (lastWayMarker != newWayMarker)
          {
            goClockwise();
            lastWayMarker = newWayMarker;
          }
          return;
        }

        if (directionOfTravel < edgeLeft && directionOfTravel < edgeOpposingTarget)
        {
          newWayMarker = GO_ANTICLOCKWISE_MARKER;
          if (lastWayMarker != newWayMarker)
          {
            goClockwise();
            lastWayMarker = newWayMarker;
          }
          return;
        }


        newWayMarker = UNKNOWN_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goUnknown();
          lastWayMarker = newWayMarker;
        }
      }
    }
  }
}

void goBlackout()
{
  drawGoUnknown(false);
  drawGoClockwise(false);
  drawGoAntiClockwise(false);
  drawGoAhead(false);
  drawGoTurnAround(false);
}

void goAhead()
{
  drawGoUnknown(false);
  drawGoClockwise(false);
  drawGoAntiClockwise(false);
  drawGoTurnAround(false);
  drawGoAhead(true);
}

void goTurnAround()
{
  drawGoUnknown(false);
  drawGoClockwise(false);
  drawGoAntiClockwise(false);
  drawGoAhead(false);
  drawGoTurnAround(true);
}

void goClockwise()
{
  drawGoUnknown(false);
  drawGoAhead(false);
  drawGoAntiClockwise(false);
  drawGoTurnAround(false);
  drawGoClockwise(true);
}

void goUnknown()
{
  drawGoAhead(false);
  drawGoTurnAround(false);
  drawGoClockwise(false);
  drawGoAntiClockwise(false);
  drawGoUnknown(true);
}

void goAntiClockwise()
{
  drawGoUnknown(false);
  drawGoAhead(false);
  drawGoTurnAround(false);
  drawGoClockwise(false);
  drawGoAntiClockwise(true);
}

void drawGoAhead(const bool show)
{
  uint32_t colour = (show ? TFT_GREEN : TFT_BLACK);
  const int screenWidth = M5.Lcd.width();
  const int screenHeight = M5.Lcd.height();

  M5.Lcd.fillTriangle(0, screenHeight,
                      screenWidth, screenHeight,
                      screenWidth / 2, screenHeight - 70,
                      colour);

  if (show)
  {
    M5.Lcd.setCursor(40, 220);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_BLACK, TFT_GREEN);
    M5.Lcd.print("Ahead");
  }
}

void drawGoTurnAround(const bool show)
{
  uint32_t colour = (show ? TFT_CYAN : TFT_BLACK);
  const int screenWidth = M5.Lcd.width();
  const int screenHeight = M5.Lcd.height();

  M5.Lcd.fillTriangle(0, screenHeight - 70,
                      screenWidth, screenHeight - 70,
                      screenWidth / 2, screenHeight,
                      colour);

  if (show)
  {
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_BLACK, TFT_CYAN);
    M5.Lcd.setCursor(45, 180);
    M5.Lcd.print("Turn");
    M5.Lcd.setCursor(30, 200);
    M5.Lcd.print("Around");
  }
}

void drawGoAntiClockwise(const bool show)
{
  uint32_t colour = (show ? TFT_RED : TFT_BLACK);
  const int screenWidth = M5.Lcd.width();
  const int screenHeight = M5.Lcd.height();

  M5.Lcd.fillTriangle(0, screenHeight,
                      0, screenHeight - 70,
                      screenWidth / 2, screenHeight,
                      colour);

  if (show)
  {
    M5.Lcd.setCursor(5, 220);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    M5.Lcd.print("Anti");
  }
}

void drawGoClockwise(const bool show)
{
  uint32_t colour = (show ? TFT_BLUE : TFT_BLACK);
  const int screenWidth = M5.Lcd.width();
  const int screenHeight = M5.Lcd.height();

  M5.Lcd.fillTriangle(screenWidth, screenHeight,
                      screenWidth, screenHeight - 70,
                      screenWidth / 2, screenHeight,
                      colour);
  if (show)
  {
    M5.Lcd.setCursor(96, 220);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
    M5.Lcd.print("Clk");
  }
}

void drawGoUnknown(const bool show)
{
  uint32_t colour = (show ? TFT_MAGENTA : TFT_BLACK);
  const int screenWidth = M5.Lcd.width();
  const int screenHeight = M5.Lcd.height();

  M5.Lcd.setCursor(screenWidth / 2, 190);
  M5.Lcd.setTextSize(5);
  M5.Lcd.setTextColor(colour, TFT_BLACK);
  M5.Lcd.print("!");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool getSmoothedMagHeading(float& b)
{
  float magHeading = 0;

  if (getMagHeadingTiltCompensated(magHeading) == false)
  {
    magHeading = -1.0;
    return false; // use the not initialised state to ignore this reading as NaN
  }

  s_smoothedCompassHeading[s_nextCompassSampleIndex] = magHeading;

  s_nextCompassSampleIndex = (s_nextCompassSampleIndex + 1) % s_smoothCompassBufferSize;

  if (!s_compassBufferInitialised)
  {
    // populate the entire smoothing buffer before doing any calculations or showing results
    if (s_nextCompassSampleIndex == 0)
      s_compassBufferInitialised = true;
    delay(s_compassSampleDelay);          // wait between compass readings 50ms
    return s_compassBufferInitialised;
  }

  // compute average from s_nextCompassSampleIndex % s_smoothCompassBufferSize to s_nextCompassSampleIndex-1
  magHeading = 0.0;

  bool correctForDiscontinuityAtZero = false;
  bool magHeadingInNWQuadrantFound = false;
  bool magHeadingInNEQuadrantFound = false;

  for (uint8_t index = s_nextCompassSampleIndex; index < s_nextCompassSampleIndex + s_smoothCompassBufferSize; index++)
  {
    if (s_smoothedCompassHeading[index % s_smoothCompassBufferSize] < 90.0)
      magHeadingInNEQuadrantFound = true;
    else if (s_smoothedCompassHeading[index % s_smoothCompassBufferSize] > 270.0)
      magHeadingInNWQuadrantFound = true;
  }

  float offset = (magHeadingInNWQuadrantFound && magHeadingInNEQuadrantFound ? 90.0 : 0.0);

  float shifted = 0.0;
  for (uint8_t index = s_nextCompassSampleIndex; index < s_nextCompassSampleIndex + s_smoothCompassBufferSize; index++)
  {
    shifted = s_smoothedCompassHeading[index % s_smoothCompassBufferSize] + offset;
    if (shifted >= 360.0)
      shifted -= 360.0;

    magHeading = magHeading + shifted;
  }

  magHeading = (magHeading / (float)s_smoothCompassBufferSize)  - offset;

  if (magHeading < 0.0)
    magHeading += 360.0;

  if (magHeading >= 359.5)
    magHeading = 0.0;

  b = magHeading;

  return s_compassBufferInitialised;
}


/*
   Returns the angular difference in the horizontal plane between the "from" vector and north, in degrees.
   Description of heading algorithm:
   Shift and scale the magnetic reading based on calibration data to find
   the North vector. Use the acceleration readings to determine the Up
   vector (gravity is measured as an upward acceleration). The cross
   product of North and Up vectors is East. The vectors East and North
   form a basis for the horizontal plane. The From vector is projected
   into the horizontal plane and the angle between the projected vector
   and horizontal north is returned.
*/
template <typename T> float magHeading(vector<T> from)
{
  sensors_event_t event;
  mag.getEvent(&event);
  magnetometer_vector = {event.magnetic.x, event.magnetic.y, event.magnetic.z};

  accel.getEvent(&event);
  accelerometer_vector = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

  // Important: subtract average of min and max from magnetometer calibration
  magnetometer_vector.x -= (magnetometer_min.x + magnetometer_max.x) / 2.0;
  magnetometer_vector.y -= (magnetometer_min.y + magnetometer_max.y) / 2.0;
  magnetometer_vector.z -= (magnetometer_min.z + magnetometer_max.z) / 2.0;
  // Compute east and north vectors
  vector<float> east;
  vector<float> north;
  vector_cross(&magnetometer_vector, &accelerometer_vector, &east);
  vector_normalize(&east);
  vector_cross(&accelerometer_vector, &east, &north);
  vector_normalize(&north);

  // compute heading
  float heading = atan2(vector_dot(&east, &from), vector_dot(&north, &from)) * 180.0 / PI;
  if (heading < 0.0) {
    heading += 360.0;
  }
  return heading;
}

template <typename Ta, typename Tb, typename To> void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

/*
   Returns the angular difference in the horizontal plane between a default vector and north, in degrees.
   The default vector here is the +X axis as indicated by the silkscreen.
*/
bool getMagHeadingTiltCompensated(float& tiltCompensatedHeading)
{
  float tch = magHeading((vector<int>) {
    1, 0, 0
  });

  if (isnan(tch))
  {
    tiltCompensatedHeading = 0;
    return false;
  }

  // correction applied according to my experimentation
  tiltCompensatedHeading = -tch;

  // Normalize to 0-360
  if (tiltCompensatedHeading < 0.0)
    tiltCompensatedHeading += 360.0;

  if (tiltCompensatedHeading >= 359.5)
    tiltCompensatedHeading = 0.0;

  return true;
}

bool getMagHeadingNotTiltCompensated(float& newHeading)
{
  sensors_event_t magEvent;
  mag.getEvent(&magEvent);
  float heading = (atan2(magEvent.magnetic.y, magEvent.magnetic.x) * 180.0) / PI;

  if (isnan(heading))
    return false;

  newHeading = -heading;

  if (newHeading < 0.0)
    newHeading += 360.0;

  if (newHeading >= 359.5)
    newHeading = 0.0;

  return true;
}

std::string getCardinal(float b)
{
  std::string result = "--";

  if      (b > 337.5 || b <= 22.5) result = "N  ";  // 0
  else if (b > 22.5 && b <= 67.5) result = "NE ";  // 45
  else if (b > 67.5 && b <= 112.5) result = "E  ";  // 90
  else if (b > 112.5 && b <= 157.5) result = "SE "; // 135
  else if (b > 157.5 && b <= 202.5) result = "S  "; // 180
  else if (b > 202.5 && b <= 247.5) result = "SW "; // 225
  else if (b > 247.5 && b <= 292.5) result = "W  "; // 270
  else if (b > 292.5 && b <= 337.5) result = "NW "; // 315

  return result;
}
/*
  void getTempAndHumidityAHT20(float& h, float& t)
  {
  sensors_event_t humidity, temp;
  Adafruit_TempHumidity.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
  h=humidity.relative_humidity;
  t=temp.temperature;
  }
*/

// depth in metres, temperature in C, water pressure in Bar, Altitude in m
void getDepth(float& d, float& d_t, float& d_p, float& d_a)
{
  if (!enableDepthSensor || !depthAvailable)
  {
    d = d_t = d_p = d_a = 0.0;
    return;
  }

  BlueRobotics_DepthSensor.read();

  float temp_d = BlueRobotics_DepthSensor.depth();

  if (temp_d > 100.0)  // reject outliers that seem to be occurring (of several 1000 metres)
  {
    return;
  }
  else if (temp_d < 0.0)
  {
    // correct for any negative number, eg -0.01 which will otherwise get cast to a large unsigned number
    d = 0.0;
  }
  else
  {
    d = temp_d;
  }

  d_t = BlueRobotics_DepthSensor.temperature();
  d_p = BlueRobotics_DepthSensor.pressure() / 1000.0;
  d_a = BlueRobotics_DepthSensor.altitude();
}

void getTempAndHumidityAndAirPressureBME280(float& h, float& t, float& p, float& p_a)
{
  if (humidityAvailable)
  {
    h = Adafruit_TempHumidityPressure.readHumidity();
    t = Adafruit_TempHumidityPressure.readTemperature();
    p = (float)(Adafruit_TempHumidityPressure.readPressure()) / 100.0 + pressure_correction;
    p_a = Adafruit_TempHumidityPressure.readAltitude(SEALEVELPRESSURE_HPA);
  }
  else
  {
    h = t = p = 0.0;
  }
}

///// GPS MESSAGE QUERY/SET

void sendCAS06QueryMessages(HardwareSerial &serial)
{
  /*

    Query the information type of the product. Refer to 1.5.8 for information content.
    0=Query firmware version number
    1=Query the hardware model and serial number
    2=Query the working mode of the multimode receiver MO=GB means dual mode of GPS+BDS
    3=Query the customer number of the product
    5=Query upgrade code information
  */

  // these are sent CAS026 messages and they work using grove port and HAT pins
  serial.write("$PCAS06,0*1B\r\n");   // response: $GPTXT,01,01,02,SW=URANUS5,V5.3.0.0*1D
  //    serial.write("$PCAS06,1*1A\r\n");   // response: $GPTXT,01,01,02,HW=ATGM336H,0001010379462*1F  (chip model ATGM336H)
  //    serial.write("$PCAS06,2*19\r\n");   // response: $GPTXT,01,01,02,MO=GB*77
  //    serial.write("$PCAS06,3*18\r\n");   // response: $GPTXT,01,01,02,CI=01B94154*04
  //    serial.write("$PCAS06,5*1E\r\n");   // response: $GPTXT,01,01,02,BS=SOC_BootLoader,V6.2.0.2*34
}


void sendCAS02LocationUpdateRateMessages(HardwareSerial &serial)
{
  /*
     1000 = update rate 1Hz, output per second 1
     500 = update rate 2Hz, output per second 2
     250 = update rate 4Hz, output per second 4
     200 = update rate 5Hz, output per second 5
     100 = update rate 10Hz, output per second 10

  */
  serial.write("$PCAS02,1000*2E\r\n");   // set to 1Hz response:
  //    serial.write("$PCAS02,500*1A\r\n");   // set to 2Hz response:
  //    serial.write("$PCAS02,100*1E\r\n");   // set to 10Hz response:
}

void SendCASICNavXQuery(HardwareSerial &serial)
{
  unsigned char NavXQuery[] = {0xBA, 0xCE, 0x00, 0x00, 0x06, 0x07, 0x00, 0x00, 0x06, 0x07};

  serial.write(NavXQuery, sizeof(NavXQuery));
}

void SendCASICNavXWalkingDynamicModel(HardwareSerial &serial, const bool setWalking)
{
  uint8_t d = (setWalking ? 2 : 0); // walking mode dynModel is 2, default is 0 (portable)

  uint8_t dynModelOffset = 4;
  uint8_t preamble[] = {0xBA, 0xCE, 0x2c, 0x00, 0x06, 0x07};
  uint8_t payload[] = {0, 0, 0, 1, 255, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0
                      };    // 44 bytes of payload

  payload[dynModelOffset] = d;

  uint32_t checksum = (preamble[2] << 24) + (preamble[3] << 16) + sizeof(payload);
  for (int i = 0; i < sizeof(payload); i = i + 4)
    checksum = checksum + *((uint32_t*)(payload + i));

  serial.write(preamble, sizeof(preamble));
  serial.write(payload, sizeof(payload));
  serial.write((uint8_t)(checksum && 0xFF));
  serial.write((uint8_t)(checksum >> 8 && 0xFF));
  serial.write((uint8_t)(checksum >> 16 && 0xFF));
  serial.write((uint8_t)(checksum >> 24 && 0xFF));

  // TOMORROW: need to find out if the checksum calculation is correct comparing to the Nav response and the ACK-ACK msg.
  // BEFORE SENDING ANY SET COMMANDS
}

void waitForCASICNavXResponse(HardwareSerial &serial)
{
  USB_SERIAL.println("Waiting for Navx Response Ok");

  uint8_t payloadLength = 44;
  uint8_t payload[payloadLength];

  uint8_t checksumLength = 4;
  uint8_t checksum[checksumLength];

  // length 44 == 0x002C

  uint8_t searchCriteria[] = {0xBA, 0xCE, 0x2C, 0x00, 0x06, 0x07};
  uint8_t criteriaCount = sizeof(searchCriteria);
  uint8_t next = 0;

  while (next < criteriaCount)
  {
    if (serial.available())
      next = ((serial.read() == searchCriteria[next]) ? next + 1 : 0);
  }
  USB_SERIAL.println("found all Navx Response preamble bytes");

  uint8_t nextPayload = 0;
  while (nextPayload < payloadLength)
  {
    if (serial.available())
    {
      payload[nextPayload] = serial.read();
      nextPayload++;
    }
  }
  USB_SERIAL.printf("read all payload bytes\n");

  uint8_t nextChecksumByte = 0;
  while (nextChecksumByte < checksumLength)
  {
    if (serial.available())
      checksum[nextChecksumByte++] = serial.read();
  }
  USB_SERIAL.printf("read all checksum bytes\n");

  // Verify checksum
  // LATER

  // What is in the payload?
  uint8_t mask0 = payload[3]; // MSB
  uint8_t mask1 = payload[2];
  uint8_t mask2 = payload[1];
  uint8_t mask3 = payload[0]; // LSB
  uint8_t dynamicModel = payload[4];
  uint8_t fixMode = payload[5];
  uint8_t minimumNumberofSatellites = payload[6];
  uint8_t maximumNumberofSatellites = payload[7];
  uint8_t minCNO = payload[8];
  uint8_t iniFix3D = payload[10];
  int8_t minElev = payload[11];

  USB_SERIAL.printf("mask0: %x mask1: %x mask2: %x mask3: %x\n", mask0, mask1, mask2, mask3);
  USB_SERIAL.printf("dyModel: %i  fixMode: %i  minSat: %i  maxSat: %i\n", dynamicModel, fixMode, minimumNumberofSatellites, maximumNumberofSatellites);
  USB_SERIAL.printf("minCNO: %i  iniFix3D: %i  minElev: %i\n", minCNO, iniFix3D, minElev);
}


void waitForCASICACKResponse(HardwareSerial &serial)
{
  while (true)
  {
    if (serial.available())
    {
      char nextByte = serial.read();
      if (nextByte == 0x05)
      {
        nextByte = serial.read();
        if (nextByte == 0x01)
        {
          USB_SERIAL.println("ACK-ACK Received");
          // format: 0xBA 0xCE 0x00 0x04 0x05 0x01 <class> <msgid> <reserve0> <reserve1> <4 checksum bytes>
          // class = type of information received correctly - unsigned char
          // msgid = number of correctly received message - unsigned char
          // reserve 0 and reserve1 = two bytes containing an unsigned short int

          // throw away next 8 bytes
          int i = 8;
          while (i)
          {
            if (serial.available())
            {
              serial.read();
              i--;
            }
          }
        }
        else if (nextByte == 0x00)
        {
          USB_SERIAL.println("ACK-NACK Received");
          // format: 0xBA 0xCE 0x00 0x04 0x05 0x00 <class> <msgid> <reserve0> <reserve1> <4 checksum bytes>
          // class = Type of information not received correctly - unsigned char
          // msgid = Number of incorrectly received messages - unsigned char
          // reserve0 and reserve1 = two bytes containing an unsigned short int
          // throw away next 8 bytes
          int i = 8;
          while (i)
          {
            if (serial.available())
            {
              serial.read();
              i--;
            }
          }
        }

        break;
      }
    }
  }
}


// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------

/*
  void batteryLevel()
  {
  c = M5.Axp.GetVapsData() * 1.4 / 1000;
  b = M5.Axp.GetVbatData() * 1.1 / 1000;
  //  M5.Lcd.print(b);
  battery = ((b - 3.0) / 1.2) * 100;
  if (battery > 100)
    battery = 100;
  else if (battery < 100 && battery > 9)
    M5.Lcd.print(" ");
  else if (battery < 9)
    M5.Lcd.print("  ");
  if (battery < 10)
    M5.Axp.DeepSleep();
  USB_SERIAL.print("battery: ");
  USB_SERIAL.println(battery);
  }
*/

void printDate()
{
  USB_SERIAL.print(gps.date.day());
  USB_SERIAL.print("/");
  USB_SERIAL.print(gps.date.month());
  USB_SERIAL.print("/");
  USB_SERIAL.println(gps.date.year());
}

// printTime() formats the time into "hh:mm:ss", and prints leading 0's
// where they're called for.
void printTime()
{
  USB_SERIAL.print(gps.time.hour());
  USB_SERIAL.print(":");
  if (gps.time.minute() < 10) USB_SERIAL.print('0');
  USB_SERIAL.print(gps.time.minute());
  USB_SERIAL.print(":");
  if (gps.time.second() < 10) USB_SERIAL.print('0');
  USB_SERIAL.println(gps.time.second());
}



void toggleOTAActive()
{
  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(0, 0);

  if (otaActiveListening)
  {
    asyncWebServer.end();
    M5.Lcd.println("OTA Disabled");
    otaActiveListening = false;
    delay (2000);
  }
  else
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      if (otaFirstInit == false)
      {
        asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
          request->send(200, "text/plain", "To upload firmware use /update");
        });

        AsyncElegantOTA.begin(&asyncWebServer);    // Start AsyncElegantOTA
        asyncWebServer.begin();
      }

      asyncWebServer.begin();
      M5.Lcd.printf("OTA Enabled");
      otaActiveListening = true;
    }
    else
    {
      M5.Lcd.println("Error: Enable Wifi First");
    }
    delay (2000);
  }

  M5.Lcd.fillScreen(TFT_BLACK);
}

void notifyESPNowNotActive()
{
  M5.Lcd.fillScreen(TFT_RED);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("Error: Enable ESPNow");
  delay (2000);
  M5.Lcd.fillScreen(TFT_BLACK);
}

void displayESPNowSendDataResult(const esp_err_t result)
{
  if (result == ESP_OK)
    M5.Lcd.println("Success");
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
    M5.Lcd.println("ESPNOW not Init.");
  else if (result == ESP_ERR_ESPNOW_ARG)
    M5.Lcd.println("ESPNOW Invalid Argument");
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
    M5.Lcd.println("ESPNOW Internal Error");
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
    M5.Lcd.println("ESPNOW No Memory");
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
    M5.Lcd.println("ESPNOW Peer not found.");
  else
    M5.Lcd.println("ESPNOW Unknown Error");
}

void publishToSilkySkipToNextTrack()
{
  if (ESPNowActive)
  {
    M5.Lcd.fillScreen(TFT_GREEN);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Silky: Skip to Next Track");
    // Send byte command to Silky to say skip to next track
    ESPNow_data_to_send = SILKY_ESPNOW_COMMAND_NEXT_TRACK;
    const uint8_t *peer_addr = ESPNow_slave.peer_addr;
    esp_err_t result = esp_now_send(peer_addr, &ESPNow_data_to_send, sizeof(ESPNow_data_to_send));

    displayESPNowSendDataResult(result);
    delay(1000);
    M5.Lcd.fillScreen(TFT_BLACK);
  }
  else
  {
    notifyESPNowNotActive();
  }
}

void publishToSilkyCycleVolumeUp()
{
  if (ESPNowActive)
  {
    M5.Lcd.fillScreen(TFT_GREEN);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Silky: Cycle volume up");
    // Send byte command to Silky to say skip to next track
    ESPNow_data_to_send = SILKY_ESPNOW_COMMAND_CYCLE_VOLUME_UP;
    const uint8_t *peer_addr = ESPNow_slave.peer_addr;
    esp_err_t result = esp_now_send(peer_addr, &ESPNow_data_to_send, sizeof(ESPNow_data_to_send));

    displayESPNowSendDataResult(result);
    delay(1000);
    M5.Lcd.fillScreen(TFT_BLACK);
  }
  else
  {
    notifyESPNowNotActive();
  }
}

void publishToSilkyTogglePlayback()
{
  if (ESPNowActive)
  {
    M5.Lcd.fillScreen(TFT_GREEN);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Silky: Toggle Playback");
    // Send byte command to Silky to say skip to next track
    ESPNow_data_to_send = SILKY_ESPNOW_COMMAND_TOGGLE_PLAYBACK;
    const uint8_t *peer_addr = ESPNow_slave.peer_addr;
    esp_err_t result = esp_now_send(peer_addr, &ESPNow_data_to_send, sizeof(ESPNow_data_to_send));

    displayESPNowSendDataResult(result);
    delay(1000);
    M5.Lcd.fillScreen(TFT_BLACK);
  }
  else
  {
    notifyESPNowNotActive();
  }
}




void toggleESPNowActive()
{
  if (enableESPNow)
  {
    M5.Lcd.fillScreen(TFT_ORANGE);
    M5.Lcd.setCursor(0, 0);

    if (ESPNowActive == false)
    {
      if (otaActiveListening)
        toggleOTAActive();

      bool disabledWiFi = false;

      if (WiFi.status() == WL_CONNECTED)
      {
        toggleWiFiActive();
        disabledWiFi = true;
      }

      connectESPNow();
      ESPNowActive = true;

      if (disabledWiFi)
        M5.Lcd.printf("Wifi Disabled\nESPNow Enabled");
      else
        M5.Lcd.printf("ESPNow Enabled");

      delay (2000);
    }
    else
    {
      // disconnect ESPNow;
      TeardownESPNow();
      ESPNowActive = false;

      M5.Lcd.printf("ESPNow Disabled");
      delay (2000);
    }
  }
}

void toggleWiFiActive()
{
  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(0, 0);

  if (WiFi.status() == WL_CONNECTED)
  {
    if (otaActiveListening)
    {
      asyncWebServer.end();
      M5.Lcd.println("OTA Disabled");
      otaActiveListening = false;
    }

    WiFi.disconnect();
    ssid_connected = ssid_not_connected;
    M5.Lcd.printf("Wifi Disabled");
    delay (2000);
  }
  else
  {
    M5.Lcd.printf("Wifi Connecting");

    // startup Wifi only
    if (!connectWiFiNoOTA(ssid_1, password_1, label_1, timeout_1))
      if (!connectWiFiNoOTA(ssid_2, password_2, label_2, timeout_2))
        connectWiFiNoOTA(ssid_3, password_3, label_3, timeout_3);
  }

  M5.Lcd.fillScreen(TFT_BLACK);
}

void toggleUptimeGlobalDisplay()
{
  enableGlobalUptimeDisplay = !enableGlobalUptimeDisplay;

  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(0, 0);
  if (enableGlobalUptimeDisplay)
    M5.Lcd.println("Uptime On");
  else
    M5.Lcd.println("Uptime Off");

  delay (2000);

  M5.Lcd.fillScreen(TFT_BLACK);
}


void toggleUplinkMessageProcessAndSend()
{
  enableUplinkComms = !enableUplinkComms;

  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(0, 0);
  if (enableUplinkComms)
    M5.Lcd.println("Uplink On");
  else
    M5.Lcd.println("Uplink Off");

  delay (2000);

  M5.Lcd.fillScreen(TFT_BLACK);
}

void shutdownIfUSBPowerOff()
{
  if (M5.Axp.GetVBusVoltage() < minimumUSBVoltage)
  {
    if (USBVoltageDropTime == 0)
      USBVoltageDropTime = millis();
    else
    {
      if (millis() > USBVoltageDropTime + milliSecondsToWaitForShutDown)
      {
        // initiate shutdown after 500ms.
        delay(500);
        fadeToBlackAndShutdown();
      }
    }
  }
  else
  {
    if (USBVoltageDropTime != 0)
      USBVoltageDropTime = 0;
  }
}

void fadeToBlackAndShutdown()
{
  for (int i = 14; i > 6; i--)
  {
    M5.Axp.ScreenBreath(i);             // 7-14 fade to black
    delay(100);
  }

  M5.Axp.PowerOff();
}

bool connectESPNow()
{
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnESPNowDataSent);
  return true;
}

bool connectWiFiNoOTA(const char* _ssid, const char* _password, const char* label, uint32_t timeout)
{
  bool forcedCancellation = false;
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(2);
  bool connected = false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid, _password);

  // Wait for connection for max of timeout/1000 seconds
  M5.Lcd.printf("%s Wifi", label);
  int count = timeout / 500;
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {
    // check for cancellation button - top button.
    updateButtonsAndBuzzer();

    if (p_primaryButton->isPressed()) // Connect to Wifi: cancel connection attempts
    {
      forcedCancellation = true;
      break;
    }

    M5.Lcd.print(".");
    delay(500);
  }
  M5.Lcd.print("\n\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    M5.Lcd.setRotation(0);
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setCursor(0, 155);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("%s\n\n", WiFi.localIP().toString());
    M5.Lcd.println(WiFi.macAddress());
    connected = true;
    ssid_connected = _ssid;

    M5.Lcd.qrcode("http://" + WiFi.localIP().toString() + "/update", 0, 0, 135);

    updateButtonsAndBuzzer();

    if (p_secondButton->isPressed()) // Connect to Wifi: pause QR for 20 seconds
    {
      M5.Lcd.print("\n20 second pause");
      delay(20000);
    }
  }
  else
  {
    if (forcedCancellation)
      M5.Lcd.print("\n     Cancelled\n Connection Attempts");
    else
      M5.Lcd.print("No Connection");
  }

  delay(1000);

  M5.Lcd.fillScreen(TFT_BLACK);

  return connected;
}

bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout)
{
  bool forcedCancellation = false;
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(2);
  bool connected = false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid, _password);

  // Wait for connection for max of timeout/1000 seconds
  M5.Lcd.printf("%s Wifi", label);
  int count = timeout / 500;
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {
    // check for cancellation button - top button.
    updateButtonsAndBuzzer();

    if (p_primaryButton->isPressed()) // Connect to Wifi: cancel connection attempts
    {
      forcedCancellation = true;
      break;
    }

    M5.Lcd.print(".");
    delay(500);
  }
  M5.Lcd.print("\n\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "To upload firmware use /update");
    });

    AsyncElegantOTA.begin(&asyncWebServer);    // Start AsyncElegantOTA
    asyncWebServer.begin();

    M5.Lcd.setRotation(0);
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setCursor(0, 155);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("%s\n\n", WiFi.localIP().toString());
    M5.Lcd.println(WiFi.macAddress());
    connected = true;
    ssid_connected = _ssid;

    M5.Lcd.qrcode("http://" + WiFi.localIP().toString() + "/update", 0, 0, 135);

    updateButtonsAndBuzzer();

    if (p_secondButton->isPressed())  // Connect to Wifi: pause QR for 20 seconds
    {
      M5.Lcd.print("\n20 second pause");
      delay(20000);
    }
  }
  else
  {
    if (forcedCancellation)
      M5.Lcd.print("\n     Cancelled\n Connection Attempts");
    else
      M5.Lcd.print("No Connection");
  }

  delay(1000);

  M5.Lcd.fillScreen(TFT_BLACK);

  return connected;
}

void readAndTestGoProButtons()
{
  BtnGoProTop.read();
  BtnGoProSide.read();

  bool btnTopPressed = BtnGoProTop.pressedFor(15);
  bool btnSidePressed = BtnGoProSide.pressedFor(15);

  if (btnTopPressed && btnSidePressed)
  {
    sideCount++;
    topCount++;
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.printf("TOP+SIDE %hu %hu", topCount, sideCount);
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.print("           \n");
    M5.Lcd.print("           \n");
  }
  else if (btnTopPressed)
  {
    topCount++;
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.printf("TOP %hu", topCount);
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.print("           \n");
    M5.Lcd.print("           \n");
  }
  else if (btnSidePressed)
  {
    sideCount++;
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.printf("SIDE %hu", sideCount);
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.print("           \n");
    M5.Lcd.print("           \n");
  }
}

float ESP32_hallRead()  // ESP32 hall value read.
{
  float value = 0;
  int count   = 100;  // was 400
  // mean value filter.
  for (int n = 0; n < count; n++) value += hallRead();

  if (hallOffset == 0)
  {
    hallOffset = value / count * 10;
    return hallOffset;
  }
  else
  {
    return value / count * 10 - hallOffset;
  }
}

bool isMagnetPresentHallSensor()
{
  bool result = false;

  float sample = ESP32_hallRead();

  if (sample < magnetHallReadingForReset || sample > -magnetHallReadingForReset)
  {
    result = true;
    // hall sensor event detected
    //     esp_restart();
    printf("Magnet %.0f\n", sample);
  }
  return result;
}


#ifdef INCLUDE_QUBITRO_AT_COMPILE_TIME
bool qubitro_connect()
{

  bool success = true;

  if (connectToQubitro && WiFi.status() == WL_CONNECTED)
  {
    qubitro_mqttClient.setId(qubitro_device_id);
    qubitro_mqttClient.setDeviceIdToken(qubitro_device_id, qubitro_device_token);

    Serial.println("Connecting to Qubitro...");

    if (!qubitro_mqttClient.connect(qubitro_host, qubitro_port))
    {
      Serial.print("Connection failed. Error code: ");
      Serial.println(qubitro_mqttClient.connectError());
      Serial.println("Visit docs.qubitro.com or create a new issue on github.com/qubitro");
      success = false;
    }
    else
    {
      Serial.println("Connected to Qubitro.");
    }

    qubitro_mqttClient.subscribe(qubitro_device_id);
  }
  else
  {
    success = false;
  }

  return success;
}

void buildBasicTelemetryMessage(char* payload)
{
  sprintf(payload, "{\"lat\":%f,\"lng\":%f}",  gps.location.lat(), gps.location.lng());
}

void buildFullFatBonzaTelemetryMessage(char* payload)
{
  // lat long as coordinates
  sprintf(payload, "{\"UTC time\":\"%02d:%02d:%02d\",\"UTC date\":\"%02d:%02d:%02d\",\"on_seconds\":%hu,\"coordinates\":[%f,%f],\"alt\":%f,\"sats\":%lu,\"hdop\":%u,\"gps_course\":%f,\"speed\":%f,\"temp\":%f,\"humid\":%f,\"pressure\":%f,\"heading_to_target\":%f,\"distance_to_target\":%f,\"magnetic_heading_comp\":%f,\"course_heading\":%f,\"course_distance\":%f,\"mag_x\":%f,\"mag_y\":%f,\"mag_z\":%f,\"acc_x\":%f,\"acc_y\":%f,\"acc_z\":%f,\"fix_count\":%u,\"side_count\":%u,\"top_count\":%u,\"usb_v\":%f,\"usb_i\":%f}",
          gps.time.hour(), gps.time.minute(), gps.time.second(),
          gps.date.day(), gps.date.month(), gps.date.year(),
          millis() / 1000,
          gps.location.lat(),
          gps.location.lng(),
          gps.altitude.meters(),
          gps.satellites.value(),
          gps.hdop.hdop(),
          gps.course.deg(),
          gps.speed.mph(),
          temperature,
          humidity,
          water_pressure,
          heading_to_target,
          distance_to_target,
          magnetic_heading,
          journey_course,
          journey_distance,
          magnetometer_vector.x,
          magnetometer_vector.y,
          magnetometer_vector.z,
          accelerometer_vector.x,
          accelerometer_vector.y,
          accelerometer_vector.z,
          fixCount,
          sideCount,
          topCount,
          M5.Axp.GetVBusVoltage(),
          M5.Axp.GetVBusCurrent()
         );

}

bool uploadTelemetryToQubitro()
{
  bool success = false;

  if (millis() < last_qubitro_upload + qubitro_upload_duty_ms)
  {
    return true;
  }
  else
  {
    last_qubitro_upload = millis();
  }

  if (connectToQubitro)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      if (qubitro_mqttClient.connectError() == SUCCESS)
      {
        char qubitro_payload[4096];
        buildFullFatBonzaTelemetryMessage(qubitro_payload);
        //Serial.println(qubitro_payload);

        qubitro_mqttClient.poll();
        qubitro_mqttClient.beginMessage(qubitro_device_id);
        qubitro_mqttClient.print(qubitro_payload);
        int endMessageResult = qubitro_mqttClient.endMessage();
        if (endMessageResult == 1)
        {
          success = true;
          Serial.printf("Qubitro Client sent message %s\n", qubitro_payload);
        }
        else
        {
          Serial.printf("Qubitro Client failed to send message, EndMessage error: %d\n", endMessageResult);
        }
      }
      else
      {
        Serial.printf("Qubitro Client error status %d\n", qubitro_mqttClient.connectError());
      }
    }
    else
    {
      Serial.printf("Q No Wifi\n");
    }
  }
  else
  {
    Serial.printf("Q Not On\n");
  }

  return success;
}
#endif


#ifdef INCLUDE_SMTP_AT_COMPILE_TIME
void sendTestByEmail()
{
  ESP_Mail_Session session;

  session.server.host_name = smtpServer;
  session.server.port = smtpPort;
  session.login.email = smtpSenderEmail;
  session.login.password = smtpSenderPassword;
  session.login.user_domain = "";

  if (!smtp.connect(&session))
  {
    Serial.println("Error connecting to SMTP, " + smtp.errorReason());
    return;
  }

  SMTP_Message emailMessage;

  emailMessage.sender.name = "Mercator Origins";
  emailMessage.sender.email = smtpSenderEmail;
  emailMessage.subject = "Mercator Origins Test Email";
  emailMessage.addRecipient("BluepadLabs", smtpRecepientEmail);

  //Send HTML message
  String htmlMsg = "<div style=\"color:#FF0000;\"><h1>Hello Bluepad Labs!</h1><p>This is a test email from Mercator Origins.</p></div>";
  emailMessage.html.content = htmlMsg.c_str();
  emailMessage.html.content = htmlMsg.c_str();
  emailMessage.text.charSet = "us-ascii";
  emailMessage.html.transfer_encoding = Content_Transfer_Encoding::enc_7bit;

  if (!MailClient.sendMail(&smtp, &emailMessage))
    Serial.println("Error sending Email, " + smtp.errorReason());
}

void sendLocationByEmail()
{
  ESP_Mail_Session session;

  session.server.host_name = SMTP_SERVER ;
  session.server.port = SMTP_PORT;
  session.login.email = SENDER_EMAIL;
  session.login.password = SENDER_PASSWORD;
  session.login.user_domain = "";

  if (!smtp.connect(&session))
  {
    Serial.println("Error connecting to SMTP, " + smtp.errorReason());
    return;
  }
  else
  {
    Serial.println("Connected to SMTP Ok");
  }
  SMTP_Message emailMessage;

  emailMessage.sender.name = "Mercator Origins";
  emailMessage.sender.email = smtpSenderEmail;
  emailMessage.subject = "Mercator Origins Location Fix";
  emailMessage.addRecipient("BluepadLabs", smtpRecipientEmail);

  //Send HTML message
  String htmlMsg = "<div style=\"color:#FF0000;\"><h1>Hello BluePad Labs!</h1><p>This is a location email sent from Mercator Origins</p></div>";
  emailMessage.html.content = htmlMsg.c_str();
  emailMessage.html.content = htmlMsg.c_str();
  emailMessage.text.charSet = "us-ascii";
  emailMessage.html.transfer_encoding = Content_Transfer_Encoding::enc_7bit;

  if (!MailClient.sendMail(&smtp, &emailMessage))
    Serial.println("Error sending Email, " + smtp.errorReason());
  else
    Serial.println("Error sending Email, " + smtp.errorReason());

}
#endif

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
    ESPNowActive = true;
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

bool TeardownESPNow()
{
  bool result = false;

  if (enableESPNow && ESPNowActive)
  {
    WiFi.disconnect();
    ESPNowActive = false;
    result = true;
  }
  return result;
}



char ESPNowDiagBuffer[256];

// callback when data is sent from Master to Slave
void OnESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

  if (status == ESP_NOW_SEND_SUCCESS)
  {
    ESPNowMessagesDelivered++;
    sprintf(ESPNowDiagBuffer, "Del Ok %hu ", ESPNowMessagesDelivered);
    // flash RED LED once
    digitalWrite(RED_LED_GPIO, LOW);
    delay(500);
    digitalWrite(RED_LED_GPIO, HIGH);
  }
  else
  {
    ESPNowMessagesFailedToDeliver++;
    sprintf(ESPNowDiagBuffer, "Del Fail %hu ", ESPNowMessagesFailedToDeliver);
    // flash RED LED three times
    digitalWrite(RED_LED_GPIO, LOW);
    delay(500);
    digitalWrite(RED_LED_GPIO, HIGH);
    delay(500);
    digitalWrite(RED_LED_GPIO, LOW);
    delay(500);
    digitalWrite(RED_LED_GPIO, HIGH);
    delay(500);
    digitalWrite(RED_LED_GPIO, LOW);
    delay(500);
    digitalWrite(RED_LED_GPIO, HIGH);
  }

  tb_display_print_String(ESPNowDiagBuffer);
}


// Scan for slaves in AP mode
void ESPNowScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  // reset on each scan
  bool slaveFound = 0;
  memset(&ESPNow_slave, 0, sizeof(ESPNow_slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (ESPNOW_PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            ESPNow_slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        ESPNow_slave.channel = ESPNOW_CHANNEL; // pick a channel
        ESPNow_slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
    //    tb_display_print_String("Slave Found, processing...");
  } else {
    Serial.println("Slave Not Found, trying again.");
    //  tb_display_print_String("No Slave, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool ESPNowManageSlave() {
  if (ESPNow_slave.channel == ESPNOW_CHANNEL) {
    if (ESPNOW_DELETEBEFOREPAIR) {
      ESPNowDeletePeer();
    }

    Serial.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(ESPNow_slave.peer_addr);
    if ( exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&ESPNow_slave);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void ESPNowDeletePeer() {
  esp_err_t delStatus = esp_now_del_peer(ESPNow_slave.peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

void ESPNowSendData() {
  ESPNow_data_to_send++;

  if (ESPNow_data_to_send == 128)
    ESPNow_data_to_send = 33;

  const uint8_t *peer_addr = ESPNow_slave.peer_addr;
  Serial.print("Sending: "); Serial.println(ESPNow_data_to_send);
  esp_err_t result = esp_now_send(peer_addr, &ESPNow_data_to_send, sizeof(ESPNow_data_to_send));
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}
