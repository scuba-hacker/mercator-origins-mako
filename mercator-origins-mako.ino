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
#include <esp_wifi.h> // only for esp_wifi_set_channel()

uint16_t ESPNowMessagesDelivered = 0;
uint16_t ESPNowMessagesFailedToDeliver = 0;

uint8_t ESPNow_data_to_send = 0;

bool soundsOn = true;
const uint8_t defaultSilkyVolume = 5;     // no software gain applied - best for sounds recorded at optimal amplitude.
const uint8_t minSilkyVolume = 1;         // quietest
const uint8_t maxSilkyVolume = 5;
uint8_t silkyVolume = defaultSilkyVolume;

esp_now_peer_info_t ESPNow_audio_pod;

const int RESET_ESPNOW_SEND_RESULT = 0xFF;
esp_err_t ESPNowSendResult=(esp_err_t)RESET_ESPNOW_SEND_RESULT;


enum e_audio_action {AUDIO_ACTION_NONE, AUDIO_ACTION_NEXT_SOUND, AUDIO_ACTION_CYCLE_VOLUME, AUDIO_ACTION_SOUNDS_TOGGLE, 
AUDIO_ACTION_PLAYBACK_TOGGLE, AUDIO_ACTION_STOP_PLAYBACK, AUDIO_ACTION_SET_VOLUME, AUDIO_ACTION_ROTATE_SOUND_SET};

e_audio_action audioAction = AUDIO_ACTION_NONE;

const uint8_t ESPNOW_CHANNEL = 1;
const uint8_t ESPNOW_NO_PEER_CHANNEL_FLAG = 0xFF;
const uint8_t ESPNOW_PRINTSCANRESULTS = 0;
const uint8_t ESPNOW_DELETEBEFOREPAIR = 0;

const uint8_t SILKY_ESPNOW_COMMAND_TOGGLE_PLAYBACK = (uint8_t)'A'; // 500 ms
const uint8_t SILKY_ESPNOW_COMMAND_CYCLE_VOLUME_UP = (uint8_t)'B'; // 2000 ms
const uint8_t SILKY_ESPNOW_COMMAND_NEXT_TRACK = (uint8_t)'C';   // 5000 ms
const uint8_t SILKY_ESPNOW_COMMAND_STOP_PLAYBACK = (uint8_t)'D';   // 10000 ms
const uint8_t SILKY_ESPNOW_COMMAND_SET_VOLUME = (uint8_t)'E';

enum e_soundFX {SFX_PIANO_AHEAD='0', SFX_PIANO_BEHIND='1',SFX_PIANO_LEFT='2',SFX_PIANO_RIGHT='3',
                SFX_ORGAN_AHEAD='4', SFX_ORGAN_BEHIND='5',SFX_ORGAN_LEFT='6',SFX_ORGAN_RIGHT='7',
                SFX_PAD_AHEAD='8', SFX_PAD_BEHIND='9',SFX_PAD_LEFT=':',SFX_PAD_RIGHT=';',SFX_NONE='_'};

e_soundFX SFX_AHEAD = SFX_PIANO_AHEAD;
e_soundFX SFX_TURN_AROUND = SFX_PIANO_BEHIND;
e_soundFX SFX_ANTICLOCKWISE = SFX_PIANO_LEFT;
e_soundFX SFX_CLOCKWISE = SFX_PIANO_RIGHT;
e_soundFX SFX_UNKNOWN = SFX_NONE;

const char *soundSets[] = {"Piano Sounds", "Organ Sounds", "Pad Sounds","No Sounds",""};
const char** currentSoundSet = soundSets;

void rotateToNextGuidanceSounds()
{
  switch(SFX_AHEAD)
  {
    case SFX_PIANO_AHEAD:
    {
      SFX_AHEAD = SFX_ORGAN_AHEAD;
      SFX_TURN_AROUND = SFX_ORGAN_BEHIND;
      SFX_ANTICLOCKWISE = SFX_ORGAN_LEFT;
      SFX_CLOCKWISE = SFX_ORGAN_RIGHT;
      break;
    }
    case SFX_ORGAN_AHEAD:
    {
      SFX_AHEAD = SFX_PAD_AHEAD;
      SFX_TURN_AROUND = SFX_PAD_BEHIND;
      SFX_ANTICLOCKWISE = SFX_PAD_LEFT;
      SFX_CLOCKWISE = SFX_PAD_RIGHT;
      break;
    }
    case SFX_PAD_AHEAD:
    {
      SFX_AHEAD = SFX_NONE;
      SFX_TURN_AROUND = SFX_NONE;
      SFX_ANTICLOCKWISE = SFX_NONE;
      SFX_CLOCKWISE = SFX_NONE;
      break;
    }
    case SFX_NONE:
    {
      SFX_AHEAD = SFX_PIANO_AHEAD;
      SFX_TURN_AROUND = SFX_PIANO_BEHIND;
      SFX_ANTICLOCKWISE = SFX_PIANO_LEFT;
      SFX_CLOCKWISE = SFX_PIANO_RIGHT;
      break;
    }
    default:
    {
      SFX_AHEAD = SFX_NONE;
      SFX_TURN_AROUND = SFX_NONE;
      SFX_ANTICLOCKWISE = SFX_NONE;
      SFX_CLOCKWISE = SFX_NONE;
    }
  }
  
  if (**(++currentSoundSet) == NULL)
    currentSoundSet = soundSets;
  
  audioAction = AUDIO_ACTION_ROTATE_SOUND_SET;
}

#include "tb_display.h"

// screen Rotation values:
// 1 = Button right
// 2 = Button above
// 3 = Button left
// 4 = Button below

uint8_t tb_buffer_screen_orientation = 1;
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
enum e_uplinkMode {SEND_NO_UPLINK_MSG, SEND_TEST_UPLINK_MSG, SEND_FULL_UPLINK_MSG};

void sendNoUplinkTelemetryMessages();
void sendUplinkTestMessage();
void sendFullUplinkTelemetryMessage();

// feature switches

const e_display_brightness ScreenBrightness = BRIGHTEST_DISPLAY;
const e_uplinkMode uplinkMode = SEND_FULL_UPLINK_MSG;
bool enableUplinkComms = true;  // can be toggled through UI
const bool enableDigitalCompass = true;
const bool enableTiltCompensation = false;
const bool enableSmoothedCompass = true;
const bool enableHumiditySensor = true;
const bool enableDepthSensor = true;
const bool enableIMUSensor = true;
const bool enableNavigationGraphics = true;
const bool enableNavigationTargeting = true;
const bool enableRecentCourseCalculation = true;
bool enableGlobalUptimeDisplay = false;    // adds a timer to compass heading display so can see if a crash/reboot has happened

bool enableWifiAtStartup = false;   // set to true only if no espnow at startup
bool enableESPNowAtStartup = true;  // set to true only if no wifi at startup

bool otaActiveListening = false; // OTA updates toggle
bool otaFirstInit = false;       // Start OTA at boot if WiFi enabled

uint16_t sensor_acquisition_time = 0;
uint16_t max_sensor_acquisition_time = 0;

bool enableESPNow = true;       //
bool ESPNowActive = false;       // will be set to true on startup if set above - can be toggled through interface.

bool isPairedWithAudioPod = false;    // starts off not paired

void OnESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

// compilation switches

//#define INCLUDE_TWITTER_AT_COMPILE_TIME
//#define INCLUDE_SMTP_AT_COMPILE_TIME
//#define INCLUDE_QUBITRO_AT_COMPILE_TIME

void (*fp_sendUplinkMessage)() =  ( uplinkMode == SEND_NO_UPLINK_MSG ? &sendNoUplinkTelemetryMessages :
                                  ( uplinkMode == SEND_TEST_UPLINK_MSG ? &sendUplinkTestMessage :
                                  ( uplinkMode == SEND_FULL_UPLINK_MSG ? &sendFullUplinkTelemetryMessage : &sendNoUplinkTelemetryMessages)));

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

const uint32_t disabledTempDisplayEndTime = 0xFFFFFFFF;
uint32_t showTempDisplayEndTime = disabledTempDisplayEndTime;
const uint32_t showTempDisplayHoldDuration = 5000;
const uint32_t showTempAudioTestDisplayHoldDuration = 2000;

char uplink_preamble_pattern[] = "MBJAEJ";
char uplink_preamble_pattern2[] = "MBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJMBJAEJ";

char uplinkTestMessages[][6] = {"MSG0 ", "MSG1 ", "MSG2 ", "MSG3 "};
char newWayMarkerLabel[2];
char directionMetricLabel[2];

const uint32_t minimumExpectedTimeBetweenFix = 1000;  // 1 second

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

uint16_t telemetryMessage[100]; // 200 bytes

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
char compassCalibrationDisplayLabel[] = "CC";
char surveyDisplayLabel[] = "SV";
char thisTargetDisplayLabel[] = "TT";
char nextWaypointDisplayLabel[] = "NT";
char audioActionDisplayLabel[] = "AA";
char undefinedDisplayLabel[] = "??";

uint32_t recordHighlightExpireTime = 0;
uint32_t recordHighlightDisplayDuration = 10000;
bool     recordSurveyHighlight = false;

class navigationWaypoint
{
//    static const uint8_t maxLabelLength = 24;

  public:
    char*  _label;
    double _lat;
    double _long;

    navigationWaypoint()
    {
      _label[0] = NULL;
      _lat = _long = 0.0;
    }

    navigationWaypoint(char*  label, double latitude, double longitude) : _label(label), _lat(latitude), _long(longitude)
    {
    }
};
/*
const uint8_t waypointCount = 14;
const uint8_t waypointExit = 0;

// TBC means location to be confirmed by navigating to the feature on another dive
// ^ means confirmed feature location
navigationWaypoint diveOneWaypoints[waypointCount] =
{
  [0] =  { ._label = "Mid\nJetty^\n", ._lat = 51.459547, ._long = -0.547461},  // from Bing or google - TBC
  [1] =  { ._label = "Search\nLight\nBoat^\n", ._lat = 51.459919, ._long = -0.547681},  // Lady of Kent, from Neo-6M - Dive 2 on 21 July -  TBC
  [2] =  { ._label = "Cement\nMixer^\n", ._lat = 51.460038, ._long = -0.547882},  // from Neo-6M - Dive 2 on 21 July - TBC
  [3] =  { ._label = "Tyre^\n", ._lat = 51.460053, ._long = -0.548184},  // from Neo-6M Dive 2 on 21 July - TBC
  [4] =  { ._label = "Cafe\nJetty^\n", ._lat = 51.460015, ._long = -0.548316},    // from Bing or google - jetty near dive centre - TBC
  [5] =  { ._label = "Container\nRusty^\n8m", ._lat = 51.460192, ._long = -0.548283},  // from Neo-6M Dive 2 on 21 July - TBC
  [6] =  { ._label = "Bus^", ._lat = 51.460073, ._long = -0.548515},        // Got from Bing, confirmed with Neo-6M
  [7] =  { ._label = "Confined\nArea\n(TBC)", ._lat = 51.459999, ._long = -0.548590}, // Got from Google Maps - TBC
  [8] =  { ._label = "Container\nNear\nBus\n2.5m^", ._lat = 51.460014, ._long = -0.548735},  // Got from Bing, confirmed with Neo-6M
  [9] =  { ._label = "Unknown 1\n(TBC)", ._lat = 51.460069, ._long = -0.548567},  // from Neo-6M Dive 2 on 21 July - TBC
  [10] = { ._label = "Unknown 2\n(TBC)", ._lat = 51.460130, ._long = -0.548850},  // from Neo-6M Dive 2 on 21 July - TBC
// fail M5 GPS  [11] = { ._label = "Reliant\nScimatar\nCar\n(TBC)", ._lat = 51.46016, ._long = -0.54873},  // Got from Dive 2 21-07-2023 with Neo-6M, TBC
// fail M5 GPS  [12] = { ._label = "Caves\n7m(TBC M5)", ._lat = 51.460725, ._long = -0.548896},  // Got from Dive 1 21-07-2023 with M5, TBC
// fail M5 GPS  [13] = { ._label = "Multi\nBoat\nWrecks\n(TBC M5)", ._lat = 51.460843, ._long = -0.548369},  // Got from Dive 1 21-07-2023 with M5, TBC
  [11] = { ._label = "Plane^\n", ._lat = 51.459745, ._long = -0.546649},   // Got from bing  - plane
  [12] = { ._label = "Old\nJetty\n(TBC)",  ._lat = 51.459280, ._long = -0.547084},  // from Bing or google - TBC
  [13] = { ._label = "The\nGaff", ._lat = 51.391231, ._long = -0.287616}
};

*/



/*
// dives on 1st August Solo

const uint8_t waypointCount = 22;
const uint8_t waypointExit = 0;

navigationWaypoint diveOneWaypoints[waypointCount] =
{
  [0] =  { ._label = "-0Mid\nJetty^\n", ._lat = 51.459547, ._long = -0.547461},
  [1] = { ._label = "--Old\nJetty\n(TBC)",  ._lat = 51.459280, ._long = -0.547084},
  [2] = { ._label = "17a-near pipe", ._lat = 51.459142, ._long = -0.5460264},
  [3] = { ._label = "10b-pipe mid?", ._lat = 51.4591603333333, ._long = -0.545986222222222},
  [4] = { ._label = "09c-pipe?", ._lat = 51.4591611666667, ._long = -0.545925166666667},
  [5] = { ._label = "18d-near pipe", ._lat = 51.4591311666667, ._long = -0.545924333333333},
  [6] = { ._label = "16e-north of pipe?",  ._lat = 51.4591815, ._long = -0.545989333333333},
  [7] = { ._label = "18d-near pipe", ._lat = 51.4591311666667, ._long = -0.545924333333333},
  [8] = { ._label = "16e-north of pipe?",  ._lat = 51.4591815, ._long = -0.545989333333333},
  [9] = { ._label = "15f-orka van?", ._lat = 51.4592124,._long = -0.5458994},
  [10] = { ._label = "08g-plane^", ._lat = 51.459753,._long = -0.546650142857143},
  [11] = { ._label = "07h-taxi^", ._lat = 51.459729, ._long = -0.546992857142857},
  [12] = { ._label = "-01i-DayBoat", ._lat = 51.459799, ._long = -0.547376555555556},
  [13] = { ._label = "-06j-middle?", ._lat = 51.4601315714286, ._long = -0.547417857142857},
  [14] = { ._label = "-14k-far side mid jetty", ._lat = 51.4602073333333, ._long = -0.546787666666667},
  [15] = { ._label = "-03l-nr bus?", ._lat = 51.4600835714286, ._long = -0.548664428571429},
  [16] = { ._label = "02m-nr bus?", ._lat = 51.4600555714286, ._long = -0.548741571428571},
  [17] = { ._label = "11n-nr bus?", ._lat = 51.4600452, ._long = -0.5488188},
  [18] = { ._label = "04o-way to caves?", ._lat = 51.4601345714286, ._long = -0.548862428571429},
  [19] = { ._label = "12p-way to caves?", ._lat = 51.460347, ._long = -0.5489195},
  [20] = { ._label = "13q-cavebike?",  ._lat = 51.460893, ._long = -0.548704666666667},
  [21] = { ._label = "05r-cave?", ._lat = 51.460947625, ._long = -0.54878325}
};
*/

/*
const uint8_t waypointCount = 20;
const uint8_t waypointExit = 19;

navigationWaypoint diveOneWaypoints[waypointCount] =
{
  [0] = { ._label = "Start", ._lat = 51.3915287, ._long = -0.2874116},
  [1] = { ._label = "Manor-Pine", ._lat = 51.3914433, ._long = -0.2868819},
  [2] = { ._label = "Pine Gardens School", ._lat = 51.3923969, ._long = -0.2855411},
  [3] = { ._label = "Pine-Queens", ._lat = 51.3917307, ._long = -0.2834097},
  [4] = { ._label = "Pine-Raeburn", ._lat = 51.3917649, ._long = -0.2820406},
  [5] = { ._label = "Stirling-Hogsmill", ._lat = 51.3917349, ._long = -0.2798634},
  [6] = { ._label = "Elmbridge Ave", ._lat = 51.3962488, ._long = -0.2769438},
  [7] = { ._label = "SW Corner", ._lat = 51.3971483, ._long = -0.2758105},
  [8] = { ._label = "Path Junction", ._lat = 51.3985988, ._long = -0.275251},
  [9] = { ._label = "Half Way Point", ._lat = 51.3987829, ._long = -0.2746287},
  [10] = { ._label = "Centre Field", ._lat = 51.3980995, ._long = -0.2747807},
  [11] = { ._label = "Green Lane Corner", ._lat = 51.3976176, ._long = -0.2733913},
  [12] = { ._label = "Elmbridge Roundabout", ._lat = 51.397113, ._long = -0.2769533},
  [13] = { ._label = "Berrylands Pub", ._lat = 51.3977556, ._long = -0.2803758},
  [14] = { ._label = "Chiltern Roundabout", ._lat = 51.3961794, ._long = -0.2820067},
  [15] = { ._label = "Pine Walk", ._lat = 51.3931675, ._long = -0.2835799},
  [16] = { ._label = "Pine-Kings", ._lat = 51.3919465, ._long = -0.2843089},
  [17] = { ._label = "Pine-Pine 2", ._lat = 51.3922843, ._long = -0.2854915},
  [18] = { ._label = "Pine-Manor 2", ._lat = 51.3913202, ._long = -0.2869346},
  [19] = { ._label = "End", ._lat = 51.3915538, ._long = -0.2873525}
};

*/
/*
// Actual recorded waypoints around the neighbourhood, not from google maps

const uint8_t waypointCount = 16;
const uint8_t waypointExit = 15;

navigationWaypoint diveOneWaypoints[waypointCount] =
{
  [0] = { ._label = "Post at\nPine/Chiltern", ._lat = 51.3920722, ._long = -0.2845362},
  [1] = { ._label = "Tree\ngreen\nhouse", ._lat = 51.3919567, ._long = -0.2840520},
  [2] = { ._label = "Tree with poster", ._lat = 51.3918198, ._long = -0.2827345},
  [3] = { ._label = "Bin Stirling", ._lat = 51.3917177, ._long = -0.2818822},
  [4] = { ._label = "Post Hogsmill", ._lat = 51.3916780, ._long = -0.2801638},
  [5] = { ._label = "Blue board Hogsmill", ._lat = 51.3923120, ._long = -0.2796272},
  [6] = { ._label = "Insect Hotel", ._lat = 51.3924743, ._long = -0.2795370},
  [7] = { ._label = "Triangle Grass", ._lat = 51.3925900, ._long = -0.2792620},
  [8] = { ._label = "Post Brick Elec", ._lat = 51.3919805, ._long = -0.2789820},
  [9] = { ._label = "New tree field", ._lat = 51.3916553, ._long = -0.2793057},
  [10] = { ._label = "Bench", ._lat = 51.3915620, ._long = -0.2795162},
  [11] = { ._label = "Allot Gate", ._lat = 51.3917410, ._long = -0.2803340},
  [12] = { ._label = "Pointy House", ._lat = 51.3917762, ._long = -0.2824163},
  [13] = { ._label = "Mushroom", ._lat = 51.3911513, ._long = -0.2872495},
  [14] = { ._label = "Silver Birch", ._lat = 51.3910167, ._long = -0.2874398},
  [15] = { ._label = "Lamp post", ._lat = 51.3907690, ._long = -0.2878912}
};
*/

/*
 * August 10th Dives 1 and 2
 * 
const uint8_t waypointCountDiveOne = 24;
const uint8_t waypointExitDiveOne = 23;

navigationWaypoint diveOneWaypoints[waypointCountDiveOne] =
{
  [0] = { ._label = "*1 Mid Jetty ", ._lat = 51.459547, ._long = -0.547461},
  [1] = { ._label = "a P31 6m", ._lat = 51.459766, ._long = -0.547347},
  [2] = { ._label = "b B38 Lifeboat 6.5m", ._lat = 51.4598136301389, ._long = -0.546991890449386},
  [3] = { ._label = "c B45 List Sharon 7.5m", ._lat = 51.4598098699702, ._long = -0.54670373432756},
  [4] = { ._label = "d 46 Plane", ._lat = 51.459745, ._long = -0.546649},
  [5] = { ._label = "e B49 Claymore 6.5m", ._lat = 51.459634435324, ._long = -0.54646635372985},
  [6] = { ._label = "f P47 6m", ._lat = 51.459399, ._long = -0.546594},
  [7] = { ._label = "g P42 6m", ._lat = 51.459491, ._long = -0.546867},
  [8] = { ._label = "h P36 6m", ._lat = 51.459555, ._long = -0.54708},
  [9] = { ._label = "i P32 6m", ._lat = 51.459658, ._long = -0.54725},
  [10] = { ._label = "j B29 Spike Boat 7m", ._lat = 51.4601315714286, ._long = -0.547417857142857},
  [11] = { ._label = "k Thorpe Boat 5.5m", ._lat = 51.4602073333333, ._long = -0.546787666666667},
  [12] = { ._label = "l P34 6m", ._lat = 51.460312, ._long = -0.547165},
  [13] = { ._label = "m My Wrecks 4m", ._lat = 51.46043825, ._long = -0.547208},
  [14] = { ._label = "n B27 Wrecks 6m", ._lat = 51.4604300973436, ._long = -0.547383365365033},
  [15] = { ._label = "o P28 6m", ._lat = 51.4602436, ._long = -0.5474192},
  [16] = { ._label = "p B21 StickUp Boat 5m", ._lat = 51.4602514070597, ._long = -0.54789158281982},
  [17] = { ._label = "q P16 Portacabin 5m", ._lat = 51.46034, ._long = -0.548173},
  [18] = { ._label = "r B13 White Boat 7m", ._lat = 51.4605198169044, ._long = -0.548421667307919},
  [19] = { ._label = "s P14 Cargo 8m", ._lat = 51.460288, ._long = -0.548292},
  [20] = { ._label = "t P9 6m", ._lat = 51.460154, ._long = -0.548687},
  [21] = { ._label = "u Bus 2m", ._lat = 51.460073, ._long = -0.548515},
  [22] = { ._label = "v P17 2m", ._lat = 51.459956, ._long = -0.54808},
  [23] = { ._label = "*1w Cafe Jetty ", ._lat = 51.460015, ._long = -0.548316}
};

const uint8_t waypointCountDiveTwo = 22;
const uint8_t waypointExitDiveTwo = 21;

navigationWaypoint diveTwoWaypoints[waypointCountDiveTwo] =
{
  [0] = { ._label = "*2 Cafe Jetty ", ._lat = 51.460015, ._long = -0.548316},
  [1] = { ._label = "a 10 Bus", ._lat = 51.460073, ._long = -0.548515},
  [2] = { ._label = "b P7 Cargo 2.5m", ._lat = 51.460014, ._long = -0.548735},
  [3] = { ._label = "c ?4 Spitfire 315d", ._lat = 0, ._long = 0},
  [4] = { ._label = "d 3 Scimitar", ._lat = 51.460347, ._long = -0.5489195},
  [5] = { ._label = "e ?5 Lightning 5.5m 12d", ._lat = 0, ._long = 0},
  [6] = { ._label = "f 13 White Boat 7m", ._lat = 51.4605198169044, ._long = -0.548421667307919},
  [7] = { ._label = "g P16 Portacab 8m", ._lat = 51.46034, ._long = -0.548173},
  [8] = { ._label = "h 21 StickUp Boat", ._lat = 51.4602514070597,  ._long = -0.54789158281982},
  [9] = { ._label = "i ?19 Chick Hutch 327d", ._lat = 0, ._long = 0},
  [10] = { ._label = "j 21 StickUp Boat", ._lat = 51.4602514070597, ._long = -0.54789158281982},
  [11] = { ._label = "k ?18 Milk Float 242d", ._lat = 0, ._long = 0},
  [12] = { ._label = "l 21 StickUp Boat", ._lat = 51.4602514070597, ._long = -0.54789158281982},
  [13] = { ._label = "m ?23 Traf Light 143d", ._lat = 0, ._long = 0},
  [14] = { ._label = "n 22 Lady of Kent 5m", ._lat = 51.4599185714286, ._long = -0.547681},
  [15] = { ._label = "o ?20 Sweet Bowl 311d", ._lat = 0, ._long = 0},
  [16] = { ._label = "p 29 Spike Boat 7m", ._lat = 51.4601315714286, ._long = -0.547417857142857},
  [17] = { ._label = "q ?35 Dragon Boat 131d", ._lat = 0, ._long = 0},
  [18] = { ._label = "r ?40 RIB 42d", ._lat = 0, ._long = 0},
  [19] = { ._label = "s 27 Wrecks 6m", ._lat = 51.4604300973436, ._long = -0.547383365365033},
  [20] = { ._label = "t ?24 Die Hard 338d", ._lat = 0, ._long = 0},
  [21] = { ._label = "*2u Cafe Jetty", ._lat = 51.460015, ._long = -0.548316}
};
*/

// Dive 1 - 7th Sep with Curt, Ben, Penny

const uint8_t waypointCountDiveOne = 20;
const uint8_t waypointExitDiveOne = 19;

navigationWaypoint diveOneWaypoints[waypointCountDiveOne] =
{
  [0] = { ._label = "Z01 Cafe Jetty", ._lat = 51.460015, ._long = -0.548316},
  [1] = { ._label = "04N Spitfire", ._lat = 51.4601028571429, ._long = -0.54883835},
  [2] = { ._label = "03N Scimitar", ._lat = 51.460347, ._long = -0.5489195},
  [3] = { ._label = "X02 Quarry Machine in Reeds", ._lat = 51.460434, ._long = -0.548921},
  [4] = { ._label = "05aN Lightning1 12d fr Scimitar (03N)", ._lat = 51.4605855, ._long = -0.548901666666667},
  [5] = { ._label = "05bN Lightning2 12d fr Scimitar (03N)", ._lat = 51.4606164444445, ._long = -0.548867111111111},
  [6] = { ._label = "02N? Sub 4m 337d from Lightning (5)", ._lat = 0, ._long = 0},
  [7] = { ._label = "01N? Canoe 3m 7d from Sub (2)", ._lat = 0, ._long = 0},
  [8] = { ._label = "12Nb? Commer Van 6m 159d fr Canoe", ._lat = 0, ._long = 0},
  [9] = { ._label = "06aN Caves Centre", ._lat = 51.460947625, ._long = -0.54878325},
  [10] = { ._label = "12Nc? Commer Van 6m 22d fr Caves", ._lat = 0, ._long = 0},  
  [11] = { ._label = "24N Half Die Hard Taxi", ._lat = 51.460773, ._long = -0.547620875},
  [12] = { ._label = "12Na? Commer Van 6m 317d fr die hard", ._lat = 0, ._long = 0},
  [13] = { ._label = "19N Chick Hutch Boat", ._lat = 51.4604027142857, ._long = -0.54804},
  [14] = { ._label = "23N Traffic Lights 7m 143d from (21B)", ._lat = 51.4600558888889, ._long = -0.547677333333333},
  [15] = { ._label = "22B Lady of Kent Search Light 5m", ._lat = 51.4599185714286, ._long = -0.547681},
  [16] = { ._label = "25N Boat In A Hole 7m ENE from (22B)", ._lat = 51.4599545, ._long = -0.54755475},
  [17] = { ._label = "20N Sweet Bowl 311d from (22B)", ._lat = 51.46020025, ._long = -0.5479775},
  [18] = { ._label = "18N Milk Float", ._lat = 51.4601745714286, ._long = -0.548058571428571},
  [19] = { ._label = "Z01 Cafe Jetty", ._lat = 51.460015, ._long = -0.548316}
};

// Dive 2 - 7th Sep with Curt, Ben, Penny

const uint8_t waypointCountDiveTwo = 14;
const uint8_t waypointExitDiveTwo = 13;

navigationWaypoint diveTwoWaypoints[waypointCountDiveTwo] =
{
  [0] = { ._label = "Z02 Mid Jetty", ._lat = 51.459547, ._long = -0.547461},
  [1] = { ._label = "30B White boat by platform", ._lat = 51.4598131428572, ._long = -0.547380285714286},
  [2] = { ._label = "38B? Lifeboat 78d 6.5m ", ._lat = 51.4598136301389, ._long = -0.546991890449386},
  [3] = { ._label = "45B List Sharon 7.5m", ._lat = 51.4598098699702, ._long = -0.54670373432756},
  [4] = { ._label = "46N Plane", ._lat = 51.459745, ._long = -0.546649},
  [5] = { ._label = "49B Claymore 6.5m", ._lat = 51.459634435324, ._long = -0.54646635372985},
  [6] = { ._label = "48N Holey Ship 191d from 49B 4.5m", ._lat = 0, ._long = 0},
  [7] = { ._label = "50cN Swim Through - crates", ._lat = 51.4592045, ._long = -0.545912625},
  [8] = { ._label = "51B Orca Van", ._lat = 51.4591431428571, ._long = -0.545936857142857},
  [9] = { ._label = "X15? Sphere Bell via Reeds to Old Jetty", ._lat = 0, ._long = 0},
  [10] = { ._label = "44N VW Van", ._lat = 51.459368, ._long = -0.546760142857143},
  [11] = { ._label = "37N Dive Bell", ._lat = 51.4595419038462, ._long = -0.547160394230769},
  [12] = { ._label = "39N London Black Cab", ._lat = 51.459729, ._long = -0.546992857142857},
  [13] = { ._label = "Z02 Mid Jetty", ._lat = 51.459547, ._long = -0.547461}
};

/*

// Dive 1 - 1st Sep with Curt and Miguel
const uint8_t waypointCountDiveOne = 14;
const uint8_t waypointExitDiveOne = 13;

navigationWaypoint diveOneWaypoints[waypointCountDiveOne] =
{
  [0] = { ._label = "Z01 Cafe Jetty", ._lat = 51.460015, ._long = -0.548316},
  [1] = { ._label = "?17P 115d, 2m", ._lat = 51, ._long = 1},
  [2] = { ._label = "15P Cargo Rusty 8m", ._lat = 51.460192, ._long = -0.548283},
  [3] = { ._label = "?14P 350d, 8m", ._lat = 51, ._long = 1},
  [4] = { ._label = "13B White Boat 7m", ._lat = 51.4605198169044, ._long = -0.548421667307919},
  [5] = { ._label = "06bN Caves Lion Entrance Caves", ._lat = 51.460817, ._long = -0.548734},
  [6] = { ._label = "06aN Caves Centre", ._lat = 51.460947625, ._long = -0.54878325},
  [7] = { ._label = "?2B Sub 300d, 4m", ._lat = 51, ._long = 1},
  [8] = { ._label = "06aN Caves Centre", ._lat = 51.460947625, ._long = -0.54878325},
  [9] = { ._label = "?5N Litning 192d, 5.5m", ._lat = 51, ._long = 1},
  [10] = { ._label = "06aN Caves Centre", ._lat = 51.460947625, ._long = -0.54878325},
  [11] = { ._label = "?12N Commer 22d, 6m", ._lat = 51, ._long = 1},
  [12] = { ._label = "?1N Canoe 339d, 3m", ._lat = 51, ._long = 1},
  [13] = { ._label = "Z01 Cafe Jetty", ._lat = 51.460015, ._long = -0.548316}
};

// Dive 2 - 1st Sep with Curt and Miguel

const uint8_t waypointCountDiveTwo = 13;
const uint8_t waypointExitDiveTwo = 12;

navigationWaypoint diveTwoWaypoints[waypointCountDiveTwo] =
{
  [0] = { ._label = "*1 Mid Jetty", ._lat = 51.459547, ._long = -0.547461},
  [1] = { ._label = "29B Dive/Spike Boat 7m", ._lat = 51.4601315714286, ._long = -0.547417857142857},
  [2] = { ._label = "?28P 0d 6m", ._lat = 51, ._long = 1},
  [3] = { ._label = "34P 6m", ._lat = 51.460312, ._long = -0.547165},
  [4] = { ._label = "?28P 235d 6m", ._lat = 51, ._long = 1},
  [5] = { ._label = "21B Sticky Up Boat 5m", ._lat = 51.4602514070597, ._long = -0.54789158281982},
  [6] = { ._label = "?23N TrafLit 143d 7m", ._lat = 51, ._long = 1},
  [7] = { ._label = "46N Plane", ._lat = 51.459745, ._long = -0.546649},
  [8] = { ._label = "49B Claymore 6.5m", ._lat = 51.459634435324, ._long = -0.54646635372985},
  [9] = { ._label = "44N VW Van", ._lat = 51.459368, ._long = -0.546760142857143},
  [10] = { ._label = "?37N Dive Bell 307d 4m", ._lat = 51, ._long = 1},
  [11] = { ._label = "33N New (Row) Boat 4.5m", ._lat = 51.4595563333333, ._long = -0.547263333333333},
  [12] = { ._label = "*1 Mid Jetty", ._lat = 51.459547, ._long = -0.547461}
};

*/

/*
// Solo dives on 30 August - repeat after previous flood with dive with Curt week of 14th August

const uint8_t waypointCountDiveOne = 23;
const uint8_t waypointExitDiveOne = 22;

navigationWaypoint diveOneWaypoints[waypointCountDiveOne] =
{
  [0] = { ._label = "*1 Mid Jetty", ._lat = 51.459547, ._long = -0.547461},
  [1] = { ._label = "a 34P Cargo 6m", ._lat = 51.460312, ._long = -0.547165},
  [2] = { ._label = "b ?40N RIB 6m 125d", ._lat = 51, ._long = 1},
  [3] = { ._label = "c ?43N Orange Boat 5.5m 80d", ._lat = 51, ._long = 1},
  [4] = { ._label = "d X08 Thorpe Boat 5.5m", ._lat = 51.4602073333333, ._long = -0.546787666666667},
  [5] = { ._label = "e 28P 6m", ._lat = 51.4602455, ._long = -0.547421},
  [6] = { ._label = "f 22B Lady Kent 5m", ._lat = 51.4599185714286, ._long = -0.547681},
  [7] = { ._label = "g ?25N Boat hole 70d close", ._lat = 51, ._long = 1},
  [8] = { ._label = "h 22B Lady Kent 5m", ._lat = 51.4599185714286, ._long = -0.547681},
  [9] = { ._label = "i ?20N Sweets 311d", ._lat = 51, ._long = 1},
  [10] = { ._label = "j 32B Sticky Up Boat", ._lat = 51.4602514070597, ._long = -0.54789158281982},
  [11] = { ._label = "k ?23N TLights 7m 143d", ._lat = 51, ._long = 1},
  [12] = { ._label = "l 17P 2m", ._lat = 51.4599656, ._long = -0.5480939},
  [13] = { ._label = "m 14P 8m", ._lat = 51.4602986, ._long = -0.5483127},
  [14] = { ._label = "n 8B The Hole", ._lat = 51.4604301666667, ._long = -0.548688166666667},
  [15] = { ._label = "o 3N Scimatar 5.5m", ._lat = 51.460347, ._long = -0.5489195},
  [16] = { ._label = "p ?5N Lightning 5.5m 12d", ._lat = 51, ._long = 1},
  [17] = { ._label = "q 06an Caves", ._lat = 51.460947625, ._long = -0.54878325},
  [18] = { ._label = "r ?2N Sub 4m 300d", ._lat = 51, ._long = 1},
  [19] = { ._label = "s ?1N Canoe 3m 7d", ._lat = 51, ._long = 1},
  [20] = { ._label = "t ?12N Commer Van 6m 340d", ._lat = 51, ._long = 1},
  [21] = { ._label = "u 13B White Boat 7m", ._lat = 51.4605198169044, ._long = -0.548421667307919},
  [22] = { ._label = "*1z Cafe Jetty", ._lat = 51.460015, ._long = -0.548316}
};

const uint8_t waypointCountDiveTwo = 18;
const uint8_t waypointExitDiveTwo = 17;

navigationWaypoint diveTwoWaypoints[waypointCountDiveTwo] =
{
  [0] = { ._label = "*2 Mid Jetty", ._lat = 51.459547, ._long = -0.547461},
  [1] = { ._label = "a X08 Thorpe Boat 5.5m", ._lat = 51.4602073333333, ._long = -0.546787666666667},
  [2] = { ._label = "b Find Rib 6m & Orange Boat 5.5m", ._lat = 51, ._long = 1},
  [3] = { ._label = "c 35N Dragon Boat", ._lat = 51.4599636666667, ._long = -0.547154333333333},
  [4] = { ._label = "d 45B Listing Sharon", ._lat = 51.4598098699702, ._long = -0.54670373432756},
  [5] = { ._label = "e 38B Lifeboat", ._lat = 51.4598136301389, ._long = -0.546991890449386},
  [6] = { ._label = "f X05 4 crates", ._lat = 51.4599018571429, ._long = -0.547141285714286},
  [7] = { ._label = "g X04 Metal Grated Box", ._lat = 51.4599582857143, ._long = -0.547648571428571},
  [8] = { ._label = "h X14 Cement Mixer", ._lat = 51.4600375, ._long = -0.5478815},
  [9] = { ._label = "i X03 Cement Mixer & Wheel", ._lat = 51.46020025, ._long = -0.5479775},
  [10] = { ._label = "j 18N Milk Float", ._lat = 51.4601745714286, ._long = -0.548058571428571},
  [11] = { ._label = "k X15 Tyre", ._lat = 51.4600531428571, ._long = -0.548183857142857},
  [12] = { ._label = "l X10 Pot in a box", ._lat = 51.459940625, ._long = -0.54852025},
  [13] = { ._label = "m X11 Seahorse Midwater", ._lat = 51.4600703333333, ._long = -0.548645666666667},
  [14] = { ._label = "n X01 Dinghy Boat", ._lat = 51.4601285, ._long = -0.5488505},
  [15] = { ._label = "o 09P Dance Platform", ._lat = 51.460154, ._long = -0.548687},
  [16] = { ._label = "p 10N Bus", ._lat = 51.460073, ._long = -0.548515},
  [17] = { ._label = "*2z Cafe Jetty", ._lat = 51.460015, ._long = -0.548316}
};
*/

const uint8_t* p_currentDiveWaypointCount = NULL;
const uint8_t* p_currentDiveWaypointExit = NULL;
navigationWaypoint* currentDiveWaypoints = NULL;

navigationWaypoint* nextWaypoint = NULL;

void switchDivePlan()
{
  if (p_currentDiveWaypointCount == NULL || p_currentDiveWaypointCount == &waypointCountDiveTwo)
  {
    p_currentDiveWaypointCount = &waypointCountDiveOne;
    p_currentDiveWaypointExit = &waypointExitDiveOne;
    currentDiveWaypoints = diveOneWaypoints;
    nextWaypoint = currentDiveWaypoints;
  }
  else
  {
    p_currentDiveWaypointCount = &waypointCountDiveTwo;
    p_currentDiveWaypointExit = &waypointExitDiveTwo;
    currentDiveWaypoints = diveTwoWaypoints;
    nextWaypoint = currentDiveWaypoints;
  }
}

enum e_way_marker {BLACKOUT_MARKER, GO_ANTICLOCKWISE_MARKER, GO_AHEAD_MARKER, GO_CLOCKWISE_MARKER, GO_TURN_AROUND_MARKER, UNKNOWN_MARKER};
enum e_direction_metric {JOURNEY_COURSE, COMPASS_HEADING};

double heading_to_target = 0, distance_to_target = 0;
double journey_lat = 0, journey_lng = 0, journey_course = 0, journey_distance = 0;
double magnetic_heading = 0;
float mag_accel_x = 0, mag_accel_y = 0, mag_accel_z = 0;
float mag_tesla_x = 0, mag_tesla_y = 0, mag_tesla_z = 0;
float humidity = 0, temperature = 0, air_pressure = 0, pressure_altitude = 0, depth = 0, water_temperature = 0, water_pressure = 0, depth_altitude = 0;
uint16_t red_sensor = 0, green_sensor = 0, blue_sensor = 0, clear_sensor = 0;
uint8_t gesture = 255, proximity = 255;
char gesture_symbol = '-';
const float pressure_correction = 0;  // mbar, calibrated against Braggs Wunderground - not used now
const float depth_correction = 0;

//const uint32_t journey_calc_period = 500;    // in milliseconds
//const uint32_t journey_min_dist = 0;          // in metres

const uint32_t journey_calc_period = 10000;    // in milliseconds
const uint32_t journey_min_dist = 5;          // in metres

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

bool           diveTimerRunning = false;
const float    minimumDivingDepthToRunTimer = 1.0;  // in metres
uint16_t       minutesDurationDiving = 0;
uint16_t       whenToStopTimerDueToLackOfDepth = 0;
uint16_t       minsToTriggerStopDiveTimer = 10;


enum  e_mako_displays 
  {SURVEY_DISPLAY, 
  NAV_COMPASS_DISPLAY, 
  NAV_COURSE_DISPLAY, 
  LOCATION_DISPLAY, 
  JOURNEY_DISPLAY, 
  AUDIO_TEST_DISPLAY,
  COMPASS_CALIBRATION_DISPLAY,
  SHOW_LAT_LONG_DISPLAY_TEMP, 
  NEXT_TARGET_DISPLAY_TEMP, 
  THIS_TARGET_DISPLAY_TEMP,
  AUDIO_ACTION_DISPLAY_TEMP};
  
const e_mako_displays first_display_rotation = SURVEY_DISPLAY;
const e_mako_displays last_display_rotation = COMPASS_CALIBRATION_DISPLAY;

e_mako_displays display_to_show = first_display_rotation;
e_mako_displays display_to_revert_to = first_display_rotation;

void switchToNextDisplayToShow()
{
  display_to_show = (e_mako_displays)((int)display_to_show + 1);

  if (display_to_show > last_display_rotation)
    display_to_show = first_display_rotation;

  M5.Lcd.fillScreen(TFT_BLACK);
  requestConsoleScreenRefresh=true;
  lastWayMarker = BLACKOUT_MARKER;
  lastWayMarkerChangeTimestamp = 0;
}

const uint8_t RED_LED_GPIO = 10;

const bool writeLogToSerial = false;

TinyGPSPlus gps;
int uart_number = 2;
HardwareSerial float_serial(uart_number);   // UART number 2: This uses Grove SCL=GPIO33 and SDA=GPIO32 for Hardware UART Tx and Rx
double Lat, Lng;
String  lat_str , lng_str;
int satellites = 0;
char internetUploadStatusGood = false;
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
  vector<T>(T xx, T yy, T zz) : x(xx),y(yy),z(zz) {}
  vector<T>() : x(0),y(0),z(0) {}
};

// Stores min and max magnetometer values from calibration

const double initial_min_mag = 1000;
const double initial_max_mag = -1000;

vector<double> calib_magnetometer_max;
vector<double> calib_magnetometer_min;
vector<double> magnetometer_max;
vector<double> magnetometer_min;
vector<double> magnetometer_vector, accelerometer_vector;
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
    *gyro_x = *gyro_y = *gyro_z = *lin_acc_x = *lin_acc_y = *lin_acc_z = *rot_acc_x = *rot_acc_y = *rot_acc_z = *IMU_temperature = 0.0;
  }
}


bool compassAvailable = true;
bool humidityAvailable = true;
bool gestureAvailable = true;
bool depthAvailable = true;
bool imuAvailable = true;


// Magnetic Compass averaging and refresh rate control
const uint8_t s_smoothedCompassBufferSize = 10;
double s_smoothedCompassHeading[s_smoothedCompassBufferSize];
uint8_t s_nextCompassSampleIndex = 0;
bool s_smoothedCompassBufferInitialised = false;
const uint16_t s_smoothedCompassSampleDelay = 100;

bool smoothedCompassCalcInProgress = false;

uint32_t s_lastCompassNotSmoothedDisplayRefresh = 0;
const uint32_t s_compassNotSmoothedHeadingUpdateRate = 150; // time between each compass update to screen 

uint32_t s_lastSendTestSerialBytesTest = 0;
const uint32_t s_sendTestSerialBytesPeriodMs = 1000;


uint32_t s_lastTempHumidityDisplayRefresh = 0;
const uint32_t s_tempHumidityUpdateRate = 1000; // time between each humidity and depth update to screen

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
uint32_t latestFixTimeStampStreamOk = 0;

// Magnetic heading calculation functions
template <typename T> double calculateTiltCompensatedHeading(vector<T> from);
template <typename Ta, typename Tb, typename To> void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
template <typename Ta, typename Tb> float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
void vector_normalize(vector<double> *a);
bool getMagHeadingTiltCompensated(double& tiltCompensatedHeading);
bool getMagHeadingNotTiltCompensated(double& heading);
bool getSmoothedMagHeading(double& b);

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
long milliSecondsToWaitForShutDown = 3000;    // When reduced to 500 there were spurious shutdowns
bool autoShutdownOnNoUSBPower = true;

bool enableButtonTestMode = false;

bool goProButtonsPrimaryControl = true;

void readAndTestGoProButtons();

void shutdownIfUSBPowerOff();

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

void setup()
{
  M5.begin();

  switchDivePlan();   // initialise to dive one

  pinMode(IR_LED_GPS_TX_PIN, OUTPUT);
  digitalWrite(IR_LED_GPS_TX_PIN, HIGH); // switch off

  M5.Lcd.setTextSize(tb_buffer_chosenTextSize);

//  tb_display_init(tb_buffer_screen_orientation, M5.Lcd.textsize);
//  tb_display_print_String("Mercator Origins - Text Buffer Enabled\n");
//  delay(1000);

  if (enableIMUSensor)
  {
    M5.Imu.Init();
  }
  else
  {
    if (writeLogToSerial)
    {
      USB_SERIAL.println("IMU Sensor Off");
    }
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

  if (enableESPNowAtStartup)
  {
    toggleESPNowActive();
  }

  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html
  //  uart_set_mode(uart_number, UART_MODE_RS485_HALF_DUPLEX);

  // re-calibrated on 28 Jul 2023 with M5 in situ in console
//  magnetometer_min = (vector<double>) { -45, -49.05, 18.15};
//  magnetometer_max = (vector<double>) { 46.5, 31.65, 106.65};

  // re-calibrated on 31 August 2023 in situ, but with magnetometer pointing 180 degrees to on 28 Jul in the console case
  magnetometer_min = (vector<double>) { -45.3, -60.6, 15.3};
  magnetometer_max = (vector<double>) { 49.8, 31.95, 110.25};

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

    if (compassAvailable && enableSmoothedCompass)
    {
      smoothedCompassCalcInProgress = true;
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
    float_serial.setRxBufferSize(512); // was 256 - must set before begin called
    float_serial.begin(UPLINK_BAUD_RATE, SERIAL_8N1, GROVE_GPS_RX_PIN, GROVE_GPS_TX_PIN);   // pin 33=rx (white M5), pin 32=tx (yellow M5), specifies the grove SCL/SDA pins for Rx/Tx
  }
  else
  {
    float_serial.setRxBufferSize(512); // was 256 - must set before begin called
    float_serial.begin(UPLINK_BAUD_RATE, SERIAL_8N1, HAT_GPS_RX_PIN, IR_LED_GPS_TX_PIN);   // pin 26=rx, 9=tx specifies the HAT pin for Rx and the IR LED for Tx (not used)
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
      USB_SERIAL.println(Adafruit_GestureSensor.readProximity());

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

void sendTestSerialBytesWhenReady()
{
  uint32_t s_lastSendTestSerialBytesTest = 0;
  const uint32_t s_sendTestSerialBytesPeriodMs = 1000;

  if (millis() > s_lastSendTestSerialBytesTest + s_sendTestSerialBytesPeriodMs)
  {
    s_lastSendTestSerialBytesTest = millis();

    float_serial.write("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU");
  }
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
      checkDivingDepthForTimer(depth);
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

  refreshDiveTimer();

#ifdef INCLUDE_QUBITRO_AT_COMPILE_TIME
// This isn't included in the build or enabled in production.
  if (connectToQubitro)
    uploadTelemetryToQubitro();
#endif
}

bool isGPSStreamOk()
{
  return (millis() - latestFixTimeStampStreamOk < minimumExpectedTimeBetweenFix);
}

bool isInternetUploadOk()
{
  return internetUploadStatusGood;
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

          latestFixTimeStampStreamOk = latestFixTimeStamp = millis();
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
          // continue reading bytes
          result = false;
        }
        else
        {
          passedChecksumCount = newPassedChecksum;

          if (gps.isSentenceGGA())      // only send uplink message back for GGA messages
          {
            // At this point a new lat/long fix has been received and is available.
            refreshAndCalculatePositionalAttributes();
  
            performUplinkTasks();

            acquireAllSensorReadings(); // compass, IMU, Depth, Temp, Humidity, Pressure
    
            checkDivingDepthForTimer(depth);
  
            refreshConsoleScreen();
    
            checkForButtonPresses();
              
            result = true;
          }
          else
          {
            result = false;
          }
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
        result = false;
      }
    }
    else
    {
      // no byte received.
      result = false;
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
  internetUploadStatusGood = (gps.altitudeUnitsGeoid.value() == 'M');
  
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
        if (journey_course >= 359.5) 
          journey_course = 0;

        journey_distance = distanceTravelled;

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
    heading_to_target = gps.courseTo(Lat, Lng, nextWaypoint->_lat, nextWaypoint->_long);
    distance_to_target = gps.distanceBetween(Lat, Lng, nextWaypoint->_lat, nextWaypoint->_long);
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
  uint32_t start_time = millis();
  uint32_t force_end_time = start_time+60;
  
  if (millis() > s_lastCompassNotSmoothedDisplayRefresh + s_compassNotSmoothedHeadingUpdateRate)
  {
    s_lastCompassNotSmoothedDisplayRefresh = millis();

    if (compassAvailable)
    {
      if (enableSmoothedCompass)
      {
        getSmoothedMagHeading(magnetic_heading);
      }
      else
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
    }
    else
    {
      magnetic_heading = 0;
    }
  }

  if (millis() > s_lastTempHumidityDisplayRefresh + s_tempHumidityUpdateRate)
  {
    s_lastTempHumidityDisplayRefresh = millis();
    //           getTempAndHumidity(humidity, temperature);
    getTempAndHumidityAndAirPressureBME280(humidity, temperature, air_pressure, pressure_altitude);
    getDepth(depth, water_temperature, water_pressure, depth_altitude);

    getM5ImuSensorData(&imu_gyro_vector.x, &imu_gyro_vector.y, &imu_gyro_vector.z,
                     &imu_lin_acc_vector.x, &imu_lin_acc_vector.y, &imu_lin_acc_vector.z,
                     &imu_rot_acc_vector.x, &imu_rot_acc_vector.y, &imu_rot_acc_vector.z,
                     &imu_temperature);
  }

  // equalise acquisition time always to 60ms
  while (millis() < force_end_time);
  
  sensor_acquisition_time = (uint16_t)(millis() - start_time);
  if (sensor_acquisition_time > max_sensor_acquisition_time)
    max_sensor_acquisition_time = sensor_acquisition_time;
}

const uint32_t buttonPressDurationToChangeScreen = 50;

void checkForButtonPresses()
{
  // ignore button presses whilst a temporary display is shown
  if (showTempDisplayEndTime != disabledTempDisplayEndTime)
    return;
    
  updateButtonsAndBuzzer();

  switch (display_to_show)
  {
    case SURVEY_DISPLAY:
    {
      if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))
      {
        switchToNextDisplayToShow();
      }
  
      if (p_secondButton->wasReleasefor(500)) // Record Highlight
      {
        recordSurveyHighlight = true;
        recordHighlightExpireTime = millis() + recordHighlightDisplayDuration;
      }
      break;
    }

    case LOCATION_DISPLAY:
    {
      if (p_primaryButton->wasReleasefor(1000)) // Location Display: toggle ota (and wifi if needed)
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
      break;
    }
    
    case JOURNEY_DISPLAY:
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
      break;
    }
    
    case AUDIO_TEST_DISPLAY:
    {
      if (p_primaryButton->wasReleasefor(10000))    // toggle between espnow and wifi
      {
        toggleESPNowActive();
      }
      if (p_primaryButton->wasReleasefor(2000))    // select next set of guidance sounds
      {
        rotateToNextGuidanceSounds();
        showTempDisplayEndTime = millis() + showTempAudioTestDisplayHoldDuration;        
        display_to_revert_to = display_to_show;
        display_to_show = AUDIO_ACTION_DISPLAY_TEMP;
      }
      else if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))  // change display screen
      {
        switchToNextDisplayToShow();
      }  

      if (p_secondButton->wasReleasefor(10000)) // toggle sounds on and off
      {
        showTempDisplayEndTime = millis() + showTempAudioTestDisplayHoldDuration;
        display_to_revert_to = display_to_show;
        display_to_show = AUDIO_ACTION_DISPLAY_TEMP;

        toggleSound();
        publishToSilkyStopPlayback();
        notifySoundsOnOffChanged();
      }
      else if (p_secondButton->wasReleasefor(5000)) // start/stop play
      {
        showTempDisplayEndTime = millis() + showTempAudioTestDisplayHoldDuration;
        display_to_revert_to = display_to_show;
        display_to_show = AUDIO_ACTION_DISPLAY_TEMP;

        publishToSilkyTogglePlayback();
      }
      else if (p_secondButton->wasReleasefor(2000)) // cycle volume up and then low at max
      {
        showTempDisplayEndTime = millis() + showTempAudioTestDisplayHoldDuration;
        display_to_revert_to = display_to_show;
        display_to_show = AUDIO_ACTION_DISPLAY_TEMP;

        publishToSilkyCycleVolumeUp();
      }
      else if (p_secondButton->wasReleasefor(50)) // Skip to next track
      {
        showTempDisplayEndTime = millis() + showTempAudioTestDisplayHoldDuration;
        display_to_revert_to = display_to_show;
        display_to_show = AUDIO_ACTION_DISPLAY_TEMP;

        publishToSilkySkipToNextTrack();
      }

      if (display_to_show == AUDIO_ACTION_DISPLAY_TEMP)
      {
        M5.Lcd.fillScreen(TFT_GREEN);
        M5.Lcd.setTextColor(TFT_BLACK,TFT_GREEN);
      }

      break;
    }
    case COMPASS_CALIBRATION_DISPLAY:
    {
      if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))  // change display screen
      {
        smoothedCompassCalcInProgress = true;
        switchToNextDisplayToShow();
      }

      if (p_secondButton->wasReleasefor(1000)) // stop calibration
      {
        M5.Lcd.print("Saving\nCalibration");  

        // save current max/min vectors to magnetometer_min and magnetometer_max
        magnetometer_min = calib_magnetometer_min;
        magnetometer_max = calib_magnetometer_max;
        
        smoothedCompassCalcInProgress = true;
        
        delay(10000);
        switchToNextDisplayToShow();
      }
      else if (p_secondButton->wasReleasefor(50)) // start calibration
      {

        M5.Lcd.fillScreen(TFT_BLACK);

        smoothedCompassCalcInProgress = false;
        
        // save current max/min vectors to magnetometer_min and magnetometer_max
        calib_magnetometer_min = vector<double>(initial_min_mag,initial_min_mag,initial_min_mag);
        calib_magnetometer_max = vector<double>(initial_max_mag,initial_max_mag,initial_max_mag);
      }

      break;
    }
    default:
    {
      if (p_primaryButton->wasReleasefor(10000)) // Nav Screens - switch to alternative dive plan
      {
        switchDivePlan();
        M5.Lcd.fillScreen(TFT_BLACK);
        delay(300);
        M5.Lcd.fillScreen(TFT_ORANGE);
        delay(300);
        M5.Lcd.fillScreen(TFT_BLACK);
        goto show_current_target;
      }
      else if (p_primaryButton->wasReleasefor(2000)) // Nav Screens : show lat long for 5 seconds
      {
        showTempDisplayEndTime = millis() + showTempDisplayHoldDuration;
        display_to_revert_to = display_to_show;
        display_to_show = SHOW_LAT_LONG_DISPLAY_TEMP;
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
        showTempDisplayEndTime = millis() + showTempDisplayHoldDuration / 3;

        // goto dive exit (the last target on the list)
        nextWaypoint = currentDiveWaypoints + *p_currentDiveWaypointExit;

        display_to_revert_to = display_to_show;
        display_to_show = NEXT_TARGET_DISPLAY_TEMP;
        M5.Lcd.fillScreen(TFT_BLACK);
      }
      else if (p_secondButton->wasReleasefor(1000))     // Nav Screens: switch to next target
      {
        showTempDisplayEndTime = millis() + showTempDisplayHoldDuration / 3;
        // head to next target, if at end of target list go to the top of the list

        if (++nextWaypoint == currentDiveWaypoints + *p_currentDiveWaypointCount)
          nextWaypoint = currentDiveWaypoints;
          
        display_to_revert_to = display_to_show;
        display_to_show = NEXT_TARGET_DISPLAY_TEMP;
        M5.Lcd.fillScreen(TFT_BLACK);
      }
      else if (p_secondButton->wasReleasefor(50))      // Nav Screens: remind diver of current target
      {
        show_current_target:
        showTempDisplayEndTime = millis() + showTempDisplayHoldDuration  / 3;

        // don't change target - remind diver of current target
        display_to_revert_to = display_to_show;
        display_to_show = THIS_TARGET_DISPLAY_TEMP;
        M5.Lcd.fillScreen(TFT_BLACK);
      }
        
      break;
    }
  }
}

void refreshConsoleScreen()
{
  switch (display_to_show)
  {
    case SURVEY_DISPLAY:
    {
      drawSurveyDisplay();
      break;
    }
    case NAV_COMPASS_DISPLAY:
    {
      drawTargetSection();
      drawCompassSection();
      break;
    }
    case NAV_COURSE_DISPLAY:
    {
      drawTargetSection();
      drawCourseSection();
      break;
    }
    case LOCATION_DISPLAY:
    {
      drawLocationStats();
      break;
    }
    case JOURNEY_DISPLAY:
    {
      drawJourneyStats();
      break;
    }
    case AUDIO_TEST_DISPLAY:
    {
      drawAudioTest();
      break;
    }
    case COMPASS_CALIBRATION_DISPLAY:
    {
      drawCompassCalibration();
      break;
    }
    case SHOW_LAT_LONG_DISPLAY_TEMP:
    {
      drawLatLong();
      break;
    }
    case NEXT_TARGET_DISPLAY_TEMP:
    {
      drawNextTarget();
      break;
    }
    case THIS_TARGET_DISPLAY_TEMP:
    {
      drawThisTarget();
      break;
    }
    case AUDIO_ACTION_DISPLAY_TEMP:
    {
      drawAudioActionDisplay();
      break;
    }
    default:
    {
      drawNullDisplay();
      break;
    }
  }

  // Used for test/debug when issues with restarts
  if (enableGlobalUptimeDisplay)
  {
    drawPowerOnTimeOverlay();
  }
}

void drawSurveyDisplay()
{
    M5.Lcd.setRotation(0);
    M5.Lcd.setCursor(15, 0);
    M5.Lcd.setTextSize(6);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);

    // Display Cyan Depth
    M5.Lcd.printf("%.1f\n", depth);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLACK);
    M5.Lcd.print("\n");
    M5.Lcd.setTextSize(4);

    if (recordHighlightExpireTime != 0)
    {
      if (millis() < recordHighlightExpireTime)
      {
        M5.Lcd.printf("-PIN-\n");
      }
      else
      {
        recordHighlightExpireTime = 0;
        M5.Lcd.print("     \n");
      }
    }
    else
    {
      if  (millis() - journey_clear_period > last_journey_commit_time || journey_distance == 0)
      {
        M5.Lcd.print("     \n");
      }
      else
      {
        M5.Lcd.printf(" %3s \n", getCardinal(journey_course).c_str());
      }
    }
        
    M5.Lcd.setTextSize(1);
    M5.Lcd.println("\n");

    M5.Lcd.setTextSize(7);

    if (diveTimerRunning == false && minutesDurationDiving == 0)    
      M5.Lcd.setTextColor(TFT_BLUE, TFT_BLACK);     // dive not started yet
    else if (diveTimerRunning == false && minutesDurationDiving > 0)
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);      // dive finished
    else if (diveTimerRunning == true && whenToStopTimerDueToLackOfDepth == 0)
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);    // dive in progress
    else if (diveTimerRunning == true && whenToStopTimerDueToLackOfDepth > 0)
      M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);   // dive in progress but not at minimum depth
    else
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);  // shouldn't get here.
      
    M5.Lcd.printf("%2hu'\n", minutesDurationDiving);

    M5.Lcd.setTextSize(3);

    if (isGPSStreamOk())
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    else
      M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);

    M5.Lcd.print("\nG");

    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

    M5.Lcd.printf(" %.0f%% ", humidity);

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    
    if (isInternetUploadOk())
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    else
      M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    
    M5.Lcd.printf("Q");
}

void drawTargetSection()
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
    if (satellites < 4.0)
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    else if (satellites < 6.0)
      M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
    else if (satellites < 10.0)
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
      M5.Lcd.setCursor(x, y);
      M5.Lcd.setTextSize(5);
      M5.Lcd.println("");
    }
    else
    {
      M5.Lcd.printf("\n*%3d", ((uint32_t)distance_to_target) % 1000);
    }
  }
}

void drawCompassSection()
{
    float directionOfTravel = magnetic_heading;

    // Display Journey Course with degrees suffix
    // this is the direction travelled in last x seconds
    // Black out the Journey Course if no recent movement
    M5.Lcd.setTextSize(5);

    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

    M5.Lcd.printf("%3.0f", directionOfTravel);

    uint16_t x = 0, y = 0, degree_offset = 0;

    x = M5.Lcd.getCursorX();
    y = M5.Lcd.getCursorY();

    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(x, y + degree_offset);

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

    M5.Lcd.setCursor(30, 146);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextFont(0);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.printf("%4.1fm", depth);

    blackout_journey_no_movement = false;
}

void drawCourseSection()
{
    float directionOfTravel = journey_course;

    // Display Journey Course with degrees suffix
    // this is the direction travelled in last x seconds
    // Black out the Journey Course if no recent movement
    M5.Lcd.setTextSize(5);

    M5.Lcd.setTextColor((blackout_journey_no_movement ? TFT_BLACK : TFT_RED), TFT_BLACK);

    M5.Lcd.printf("%3.0f", directionOfTravel);

    uint16_t x = 0, y = 0, degree_offset = 0, cardinal_offset = 0, metre_offset = 0;
    
    x = M5.Lcd.getCursorX();
    y = M5.Lcd.getCursorY();

    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(x, y + degree_offset);

    if (GPS_status == GPS_FIX_FROM_FLOAT)
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
    }
    else
    {
      // do nothing
    } 
}

void drawNextTarget()
{
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(0, 5);

  M5.Lcd.printf ("Next:\n\n(%i)\n\n%s", nextWaypoint-currentDiveWaypoints+1, nextWaypoint->_label);

  if (millis() > showTempDisplayEndTime)
  {
    showTempDisplayEndTime = disabledTempDisplayEndTime;
    display_to_show = display_to_revert_to;
    M5.Lcd.fillScreen(TFT_BLACK);
  }
}

void drawThisTarget()
{
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(0, 5);

  M5.Lcd.printf ("Towards\n\n(%i)\n\n%s", nextWaypoint-currentDiveWaypoints+1, nextWaypoint->_label);

  if (millis() > showTempDisplayEndTime)
  {
    showTempDisplayEndTime = disabledTempDisplayEndTime;
    display_to_show = display_to_revert_to;
    M5.Lcd.fillScreen(TFT_BLACK);
  }
}

void drawAudioActionDisplay()
{
  M5.Lcd.setCursor(10, 10);

  switch (audioAction)
  {
    case AUDIO_ACTION_NEXT_SOUND:
      M5.Lcd.println("Silky:\nSkip to Next Sound\n");
      displayESPNowSendDataResult(ESPNowSendResult);
      break;

    case AUDIO_ACTION_CYCLE_VOLUME:
      M5.Lcd.printf("Silky:\nCycle volume up %u\n",silkyVolume);
      displayESPNowSendDataResult(ESPNowSendResult);
      break;

    case AUDIO_ACTION_SOUNDS_TOGGLE:
      M5.Lcd.println(soundsOn ? "Silky:\nSounds On\n" : "Silky:\nSounds Off\n"); 
      break;

    case AUDIO_ACTION_PLAYBACK_TOGGLE:
      M5.Lcd.println("Silky:\nToggle Playback\n");
      displayESPNowSendDataResult(ESPNowSendResult);
      break;

    case AUDIO_ACTION_STOP_PLAYBACK:
      M5.Lcd.println("Silky:\nStop Playback\n");
      displayESPNowSendDataResult(ESPNowSendResult);
      break;

    case AUDIO_ACTION_SET_VOLUME:
      M5.Lcd.println("Silky:\nSet Volume\n");
      displayESPNowSendDataResult(ESPNowSendResult);
      break;

    case AUDIO_ACTION_ROTATE_SOUND_SET:
    {
      M5.Lcd.println("Silky:\nSound Set To\n");
      M5.Lcd.println(*currentSoundSet);
      break;
    }
    
    case AUDIO_ACTION_NONE:
      // shouldn't get here
      M5.Lcd.println("Silky:\nAudio Action None\n");
      break;

    case RESET_ESPNOW_SEND_RESULT:
      // shouldn't get here
      M5.Lcd.println("Silky:\nAudio Action Reset\n");
      break;

    default:
      // shouldn't get here
      M5.Lcd.println("Silky:\nUndefined Audio Action\n");
      break;
  }

  if (millis() > showTempDisplayEndTime)
  {
    ESPNowSendResult = RESET_ESPNOW_SEND_RESULT;
    
    showTempDisplayEndTime = disabledTempDisplayEndTime;
    display_to_show = display_to_revert_to;
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_BLACK, TFT_GREEN);
  }
}

void drawLatLong()
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
  M5.Lcd.printf("N:%.7f\n", Lat);
  M5.Lcd.printf("E:%.7f\n", Lng);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.printf("%.1fm %hu mins\n", depth, minutesDurationDiving);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  M5.Lcd.printf("%02d:%02d:%02d %02d%02d", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month());

  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  if (millis() > showTempDisplayEndTime)
  {
    showTempDisplayEndTime = disabledTempDisplayEndTime;
    display_to_show = display_to_revert_to;
    M5.Lcd.fillScreen(TFT_BLACK);
  }
}
    
void drawLocationStats()
{
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

  M5.Lcd.setCursor(5, 0);
  M5.Lcd.printf("La:%.6f   ", Lat);
  M5.Lcd.setCursor(5, 17);
  M5.Lcd.printf("Lo:%.6f   ", Lng);

  M5.Lcd.setCursor(5, 34);
  M5.Lcd.printf("Depth:%.0f m s:%hu ^%hu  ", depth,sensor_acquisition_time, max_sensor_acquisition_time);

  M5.Lcd.setCursor(5, 51);
  if (WiFi.status() == WL_CONNECTED)
  {
    M5.Lcd.printf("%s ", WiFi.localIP().toString());
    if (otaActiveListening)
      M5.Lcd.println("OTA");
    else
      M5.Lcd.println("");
  }
  else
  {
    if (ESPNowActive)
    {
      M5.Lcd.print(WiFi.macAddress());
    }
    else
      M5.Lcd.print("WiFi & ESPNow: Off");
  }

  M5.Lcd.setCursor(5, 68);
  M5.Lcd.printf("T: (%d)\n%s", (int)(nextWaypoint - currentDiveWaypoints)+1, nextWaypoint->_label);
}

void drawJourneyStats()
{
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

  M5.Lcd.setCursor(5, 0);
  M5.Lcd.printf("V:%.2fV I:%.0fmA  ", M5.Axp.GetVBusVoltage(), M5.Axp.GetVBusCurrent());
  M5.Lcd.setCursor(5, 17);


  M5.Lcd.printf("OTA:%hu Uplink:%hu", otaActiveListening,  enableUplinkComms);

  M5.Lcd.setCursor(5, 34);  
  M5.Lcd.printf("Wifi:%hu %s", WiFi.status() == WL_CONNECTED, (ESPNowActive ? "ESPNow On" : ssid_connected));
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

void drawCompassCalibration()
{
  M5.Lcd.setRotation(0);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

  if (smoothedCompassCalcInProgress == false)
  {
    M5.Lcd.printf("Calib Start\n           \n");

    sensors_event_t magEvent;

    mag.getEvent(&magEvent);
  
    if (magEvent.magnetic.x < calib_magnetometer_min.x) calib_magnetometer_min.x = magEvent.magnetic.x;
    if (magEvent.magnetic.x > calib_magnetometer_max.x) calib_magnetometer_max.x = magEvent.magnetic.x;
  
    if (magEvent.magnetic.y < calib_magnetometer_min.y) calib_magnetometer_min.y = magEvent.magnetic.y;
    if (magEvent.magnetic.y > calib_magnetometer_max.y) calib_magnetometer_max.y = magEvent.magnetic.y;
  
    if (magEvent.magnetic.z < calib_magnetometer_min.z) calib_magnetometer_min.z = magEvent.magnetic.z;
    if (magEvent.magnetic.z > calib_magnetometer_max.z) calib_magnetometer_max.z = magEvent.magnetic.z;
    
    M5.Lcd.printf("x %.3f\n  %.3f\n\n",calib_magnetometer_min.x,calib_magnetometer_max.x);
    M5.Lcd.printf("y %.3f\n  %.3f\n\n",calib_magnetometer_min.y,calib_magnetometer_max.y);
    M5.Lcd.printf("z %.3f\n  %.3f\n\n",calib_magnetometer_min.z,calib_magnetometer_max.z);
  }
  else
  {
    M5.Lcd.printf("Bottom\nbutton\nto start\ncalibration\n\n");    
  }
}

void drawAudioTest()
{
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(0, 0);

  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.printf("AUDIO TEST\nESPNow ");

  if (ESPNowActive)
    M5.Lcd.println("On");
  else
    M5.Lcd.println("Off");

  M5.Lcd.println(isPairedWithAudioPod ? "Audio Paired" : "Not Paired");

  if (isPairedWithAudioPod && soundsOn)
  {
    M5.Lcd.printf("Sounds On (%i)",silkyVolume);
  }
  else
  {
    M5.Lcd.println("Sounds Off");
  }

  if (writeLogToSerial)
  {
    USB_SERIAL.println("Toggle ESPNow: Top 10s\n");
    USB_SERIAL.println("Start/Stop Play: Side 0.5s\n");
    USB_SERIAL.println("Vol cycle: Side 2s\n");
    USB_SERIAL.println("Next Track: Side 5s\n");
  }
}

void drawNullDisplay()
{
  M5.Lcd.setCursor(5, 17);
  M5.Lcd.printf("NULL DISPLAY");
}

void drawPowerOnTimeOverlay()
{
  // overlay count up / power on time in seconds.
  M5.Lcd.setCursor(0, SCREEN_WIDTH - 15);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  M5.Lcd.printf(" Uptime: %.1f ", ((float)millis() / 1000.0));
}

void performUplinkTasks()
{
  if (enableUplinkComms)
  {
    fp_sendUplinkMessage();

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

void sendFullUplinkTelemetryMessage()
{
  sendUplinkTelemetryMessageV5();
}

enum e_user_action{NO_USER_ACTION=0x0000, HIGHLIGHT_USER_ACTION=0x0001};

uint16_t getOneShotUserActionForUplink()
{
  int userAction = NO_USER_ACTION;
  
  if (recordSurveyHighlight || recordHighlightExpireTime != 0)
  {
    // log the highlight in all messages for the time the highlight is shown on the screen (5 seconds)
    recordSurveyHighlight = false;
    userAction |= HIGHLIGHT_USER_ACTION;
  }

  return (uint16_t)userAction;
}

// send number of good messages received and number of bad messages received. 16 bit for both.
void sendUplinkTelemetryMessageV5()
{
  const uint32_t quietTimeMsBeforeUplink = 0; // disabled - always uplink on call to this method

//  delay(10);    // 10ms delay before sending
//  if (millis() > latestFixTimeStamp + quietTimeMsBeforeUplink)
//  {
    latestFixTimeStamp = CLEARED_FIX_TIME_STAMP;

    // this is 57 words, 114 bytes including checksum (56 metrics)

    // fixed format

    uint16_t uplink_length = 114;   // bytes to transmit in this message - including length and checksum. 57 x 2 byte words == 114 bytes
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
    uint16_t uplink_mako_user_action = getOneShotUserActionForUplink();

//    uint16_t uplink_mako_AXP192_temp = M5.Axp.GetTempInAXP192() * 10.0;     /// changed to number of bad lemon msg uint16_t
    uint16_t uplink_mako_bad_checksum_msgs = newFailedChecksum;

    uint16_t uplink_mako_usb_voltage = M5.Axp.GetVBusVoltage() * 1000.0;
    uint16_t uplink_mako_usb_current = M5.Axp.GetVBusCurrent() * 100.0;

    uint16_t uplink_mako_bat_voltage = M5.Axp.GetBatVoltage() * 1000.0;
    uint16_t uplink_mako_bat_charge_current = M5.Axp.GetBatChargeCurrent() * 100.0;

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

//    float uplink_mako_imu_temperature = imu_temperature * 10.0;    /// changed to number of good lemon msg uint16_t
    uint16_t uplink_mako_good_checksum_msgs = newPassedChecksum;

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
      case SHOW_LAT_LONG_DISPLAY_TEMP: displayLabel[0] = showLatLongDisplayLabel[0]; displayLabel[1] = showLatLongDisplayLabel[1]; break;
      case AUDIO_TEST_DISPLAY:  displayLabel[0] = audioTestDisplayLabel[0]; displayLabel[1] = audioTestDisplayLabel[1]; break;
      case COMPASS_CALIBRATION_DISPLAY: displayLabel[0] = compassCalibrationDisplayLabel[0]; displayLabel[1] = compassCalibrationDisplayLabel[1]; 
      case SURVEY_DISPLAY:      displayLabel[0] = surveyDisplayLabel[0]; displayLabel[1] = surveyDisplayLabel[1]; break;
      case NEXT_TARGET_DISPLAY_TEMP: displayLabel[0] = nextWaypointDisplayLabel[0]; displayLabel[1] = nextWaypointDisplayLabel[1]; break;
      case THIS_TARGET_DISPLAY_TEMP: displayLabel[0] = thisTargetDisplayLabel[0]; displayLabel[1] = thisTargetDisplayLabel[1]; break;      
      case AUDIO_ACTION_DISPLAY_TEMP: displayLabel[0] = thisTargetDisplayLabel[0]; displayLabel[1] = thisTargetDisplayLabel[1]; break;      
      default:                  displayLabel[0] = audioActionDisplayLabel[0]; displayLabel[1] = audioActionDisplayLabel[1]; break;
    }

    *(nextMetric++) = (uint16_t)(displayLabel[0]) + (((uint16_t)(displayLabel[1])) << 8); // which display is shown on the console

    *(nextMetric++) = uplink_mako_seconds_on;
    *(nextMetric++) = uplink_mako_user_action;
    *(nextMetric++) = uplink_mako_bad_checksum_msgs;
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
    *(nextMetric++) = uplink_mako_good_checksum_msgs;
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

    // calculate and store checksum by xor'ing all words, except the 
    uint16_t checksum = 0;
    uint16_t wordsToXOR = uplink_length / 2;
    for (int i = 0; i < wordsToXOR; i++)
    {
      checksum ^= telemetryMessage[i];
    }

    telemetryMessage[wordsToXOR-1] = checksum;

    float_serial.write(uplink_preamble_pattern2);

    float_serial.write((char*)telemetryMessage, uplink_length);

    // clear flags
    if (setTweetLocationNowFlag == true)
      setTweetLocationNowFlag = false;

    if (setTweetEmergencyNowFlag == true)
      setTweetEmergencyNowFlag = false;
//  }
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

const int32_t durationBetweenGuidanceSounds = 3000;

void refreshDirectionGraphic( float directionOfTravel,  float headingToTarget)
{
  if (!enableNavigationGraphics)
    return;

  // Calculate whether the diver needs to continue straight ahead,
  // rotate clockwise or rotate anticlockwise and update graphic.
  // Blacks out if no journey recorded.
  const int16_t edgeBound = 25;    // If journey course within +- 25 degrees of target heading then go ahead

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
    if (millis() - lastWayMarkerChangeTimestamp > durationBetweenGuidanceSounds)
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
        publishToSilkyPlayAudioGuidance(SFX_AHEAD);
      }
      
      else if (e1 > e2)           // scenario 4
      {
        newWayMarker = GO_TURN_AROUND_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goTurnAround();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_TURN_AROUND);
      }
      else if (o <= d && d <= e1) // scenario 2
      {
        newWayMarker = GO_CLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goClockwise();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_CLOCKWISE);
      }
      else if (e2 <= d && d <= o) // scenario 3
      {
        newWayMarker = GO_ANTICLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goAntiClockwise();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_ANTICLOCKWISE);
      }
      else if (o <= d && d <= e1) // scenario 5
      {
        newWayMarker = GO_CLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goClockwise();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_CLOCKWISE);
      }
      else if (e2 <= d && d <= o) // scenario 6
      {
        newWayMarker = GO_ANTICLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goAntiClockwise();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_ANTICLOCKWISE);
      }
      else
      {
        newWayMarker = UNKNOWN_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goUnknown();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_UNKNOWN);
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
    M5.Lcd.setCursor(40, 180);
    M5.Lcd.print("About");
    M5.Lcd.setCursor(45, 200);
    M5.Lcd.print("Turn");
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

bool getSmoothedMagHeading(double& magHeading)
{
  magHeading = 0;

  if (getMagHeadingTiltCompensated(magHeading) == false)
  {
    magHeading = -1.0;
    return false; // use the not initialised state to ignore this reading as NaN
  }

  s_smoothedCompassHeading[s_nextCompassSampleIndex] = magHeading;

  s_nextCompassSampleIndex = (s_nextCompassSampleIndex + 1) % s_smoothedCompassBufferSize;

  if (!s_smoothedCompassBufferInitialised)
  {
    // populate the entire smoothing buffer before doing any calculations or showing results
    if (s_nextCompassSampleIndex == 0)
      s_smoothedCompassBufferInitialised = true;
    return s_smoothedCompassBufferInitialised;
  }

  // compute average from s_nextCompassSampleIndex % s_smoothedCompassBufferSize to s_nextCompassSampleIndex-1
  magHeading = 0.0;

  bool correctForDiscontinuityAtZero = false;
  bool magHeadingInNWQuadrantFound = false;
  bool magHeadingInNEQuadrantFound = false;

  for (uint8_t index = s_nextCompassSampleIndex; index < s_nextCompassSampleIndex + s_smoothedCompassBufferSize; index++)
  {
    if (s_smoothedCompassHeading[index % s_smoothedCompassBufferSize] < 90.0)
      magHeadingInNEQuadrantFound = true;
    else if (s_smoothedCompassHeading[index % s_smoothedCompassBufferSize] > 270.0)
      magHeadingInNWQuadrantFound = true;
  }

  double offset = (magHeadingInNWQuadrantFound && magHeadingInNEQuadrantFound ? 90.0 : 0.0);

  double shifted = 0.0;
  for (uint8_t index = s_nextCompassSampleIndex; index < s_nextCompassSampleIndex + s_smoothedCompassBufferSize; index++)
  {
    shifted = s_smoothedCompassHeading[index % s_smoothedCompassBufferSize] + offset;
    if (shifted >= 360.0)
      shifted -= 360.0;

    magHeading = magHeading + shifted;
  }

  magHeading = (magHeading / (double)s_smoothedCompassBufferSize)  - offset;

  if (magHeading < 0.0)
    magHeading += 360.0;

  if (magHeading >= 359.5)
    magHeading = 0.0;

  return s_smoothedCompassBufferInitialised;
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
template <typename T> double calculateTiltCompensatedHeading(vector<T> from)
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
  vector<double> east;
  vector<double> north;
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

void vector_normalize(vector<double> *a)
{
  double mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

/*
   Returns the angular difference in the horizontal plane between a default vector and north, in degrees.
   The default vector here is the +X axis as indicated by the silkscreen.
*/
bool getMagHeadingTiltCompensated(double& tiltCompensatedHeading)
{
  double tch = calculateTiltCompensatedHeading((vector<int>) {1, 0, 0});  // was 1

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

  // correct for reversed dev module in gopro case - points south instead of north when north is reported.
 
  const bool magnetometerReversedInCase = true; // set to false if module points towards north when reporting north

  if (magnetometerReversedInCase)
  {
    tiltCompensatedHeading += 180.0;
    
    if (tiltCompensatedHeading >= 359.5)
      tiltCompensatedHeading -= 360.0;
  }
  
  return true;
}

bool getMagHeadingNotTiltCompensated(double& newHeading)
{
  sensors_event_t magEvent;
  mag.getEvent(&magEvent);
  double heading = (atan2(magEvent.magnetic.y, magEvent.magnetic.x) * 180.0) / PI;

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
  std::string result = "---";

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

//  d = testDepthTimer();
}

float testDepthTimer()
{
  uint32_t now = millis();
  if (now < 30000)        // 30s
    return 0; 
  else if (now < 95000)   // 1 min 35s
    return 3;
  else if (now < 110000)  // 1 min 50s
    return 0;
  else if (now < 215000)  // 3 min 35s 
    return 4;
  else
    return 0;   // Should revert to 3 minutes after a further 4 minutes. at 7 min 35s
}

void checkDivingDepthForTimer(const float& d)
{
  if (!diveTimerRunning && minutesDurationDiving == 0 && d >= minimumDivingDepthToRunTimer)
  {
    startDiveTimer();
  }
  else
  {
    if (diveTimerRunning)
    {
      if (d < minimumDivingDepthToRunTimer)
      {
        notifyNotAtDivingDepth();
      }
      else
      {
        // at diving depth so reset the time that dive timer is stopped if at < 1m depth continuously
        whenToStopTimerDueToLackOfDepth = 0;
      }
    }
  }
  refreshDiveTimer();
}

void startDiveTimer()
{
  resetRealTimeClock();
  diveTimerRunning = true;
}

// If not at diving depth for >= 10 minutes then dive timer is stopped permanently.
void notifyNotAtDivingDepth()
{
  if (diveTimerRunning)
  {
    if (whenToStopTimerDueToLackOfDepth == 0)
    {
      RTC_TimeTypeDef TimeStruct;         // Hours, Minutes, Seconds
      M5.Rtc.GetTime(&TimeStruct);
  
      whenToStopTimerDueToLackOfDepth = TimeStruct.Hours*60 + TimeStruct.Minutes + minsToTriggerStopDiveTimer;
    }
    else if (minutesDurationDiving >= whenToStopTimerDueToLackOfDepth)
    {
      minutesDurationDiving -= minsToTriggerStopDiveTimer;
      diveTimerRunning = false;
    }
  }
}

void refreshDiveTimer()
{
  if (diveTimerRunning)
  {
    RTC_TimeTypeDef TimeStruct;         // Hours, Minutes, Seconds
    M5.Rtc.GetTime(&TimeStruct);
    
    minutesDurationDiving = TimeStruct.Hours*60 + TimeStruct.Minutes;
  }
}

void resetRealTimeClock()
{
  M5.Lcd.fillScreen(BLACK);
  RTC_TimeTypeDef TimeStruct;         // Hours, Minutes, Seconds 
  TimeStruct.Hours   = 0;
  TimeStruct.Minutes = 0;
  TimeStruct.Seconds = 0;

  M5.Rtc.SetTime(&TimeStruct);

  minutesDurationDiving = 0;
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

void toggleSound()
{
 if (soundsOn)
    soundsOn = false;
  else
    soundsOn = true;  
}

void notifyESPNowNotActive()
{
  M5.Lcd.fillScreen(TFT_RED);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("Error: ESPNow inactive");
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

void publishToSilkyPlayAudioGuidance(enum e_soundFX sound)
{
  if (ESPNowActive && soundsOn && ESPNow_audio_pod.channel == ESPNOW_CHANNEL && sound != SFX_NONE)
  {
    ESPNow_data_to_send = (uint8_t)sound;
    const uint8_t *peer_addr = ESPNow_audio_pod.peer_addr;
    ESPNowSendResult = esp_now_send(peer_addr, &ESPNow_data_to_send, sizeof(ESPNow_data_to_send));

    audioAction = AUDIO_ACTION_NONE;
  }
}

void publishToSilkySkipToNextTrack()
{
  if (ESPNowActive && soundsOn && ESPNow_audio_pod.channel == ESPNOW_CHANNEL)
  {
    // Send byte command to Silky to say skip to next track
    ESPNow_data_to_send = SILKY_ESPNOW_COMMAND_NEXT_TRACK;
    const uint8_t *peer_addr = ESPNow_audio_pod.peer_addr;
    ESPNowSendResult = esp_now_send(peer_addr, &ESPNow_data_to_send, sizeof(ESPNow_data_to_send));

    audioAction = AUDIO_ACTION_NEXT_SOUND;
  }
}

void publishToSilkyCycleVolumeUp()
{
  if (ESPNowActive && soundsOn && ESPNow_audio_pod.channel == ESPNOW_CHANNEL)
  {
    if (silkyVolume == maxSilkyVolume)
      silkyVolume = minSilkyVolume;
    else
      silkyVolume++;

    publishToSilkySetVolume(silkyVolume);

    audioAction = AUDIO_ACTION_CYCLE_VOLUME;
/*
    // Send byte command to Silky to say skip to next track
    ESPNow_data_to_send = SILKY_ESPNOW_COMMAND_CYCLE_VOLUME_UP;
    const uint8_t *peer_addr = ESPNow_audio_pod.peer_addr;
    ESPNowSendResult = esp_now_send(peer_addr, &ESPNow_data_to_send, sizeof(ESPNow_data_to_send));
    audioAction = AUDIO_ACTION_CYCLE_VOLUME;
    */
  }
}

void publishToSilkySetVolume(const uint8_t newVolume)
{
  if (ESPNowActive && ESPNow_audio_pod.channel == ESPNOW_CHANNEL)
  {
    silkyVolume = newVolume;
      
    // Send byte command to Silky to say set volume to silkyVolume
    uint16_t ESPNow_word_to_send = ((uint16_t)silkyVolume << 8) | (uint16_t)SILKY_ESPNOW_COMMAND_SET_VOLUME;
    const uint8_t *peer_addr = ESPNow_audio_pod.peer_addr;
    ESPNowSendResult = esp_now_send(peer_addr, (uint8_t*)&ESPNow_word_to_send, sizeof(ESPNow_word_to_send));
    audioAction = AUDIO_ACTION_NONE;  // done on startup and no screen change needed.
  }
}

void publishToSilkyTogglePlayback()
{
  if (ESPNowActive && soundsOn && ESPNow_audio_pod.channel == ESPNOW_CHANNEL)
  {
    // Send byte command to Silky to say skip to next track
    ESPNow_data_to_send = SILKY_ESPNOW_COMMAND_TOGGLE_PLAYBACK;
    const uint8_t *peer_addr = ESPNow_audio_pod.peer_addr;
    ESPNowSendResult = esp_now_send(peer_addr, &ESPNow_data_to_send, sizeof(ESPNow_data_to_send));
    audioAction = AUDIO_ACTION_PLAYBACK_TOGGLE;
  }
}

void publishToSilkyStopPlayback()
{
  if (ESPNowActive && !soundsOn && ESPNow_audio_pod.channel == ESPNOW_CHANNEL)
  {
    // Send byte command to Silky to say skip to next track
    ESPNow_data_to_send = SILKY_ESPNOW_COMMAND_STOP_PLAYBACK;
    const uint8_t *peer_addr = ESPNow_audio_pod.peer_addr;
    ESPNowSendResult = esp_now_send(peer_addr, &ESPNow_data_to_send, sizeof(ESPNow_data_to_send));
    audioAction = AUDIO_ACTION_STOP_PLAYBACK;
  }
}

void notifySoundsOnOffChanged()
{
  if (ESPNowActive && ESPNow_audio_pod.channel == ESPNOW_CHANNEL)
  {
    ESPNowSendResult = ESP_OK;
    audioAction = AUDIO_ACTION_SOUNDS_TOGGLE;
  }
}

void toggleESPNowActive()
{
  if (enableESPNow)
  {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(2);

    bool audioPodFound = false;
    
    bool disabledWiFi = false;
    
    if (ESPNowActive == false)
    {
      if (otaActiveListening)
        toggleOTAActive();

      if (WiFi.status() == WL_CONNECTED)
      {
        toggleWiFiActive();
        disabledWiFi = true;
      }

      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(0,0);

      bool success = connectESPNow();

      if (success)
      {
        ESPNowActive = true;

        M5.Lcd.setRotation(0);
        M5.Lcd.println("  ESPNow\nSearching\n");
        if (writeLogToSerial)
          USB_SERIAL.println("Wifi\nDisabled\nESPNow\nEnabled");

        isPairedWithAudioPod = pairWithPeer(ESPNow_audio_pod,"AudioPod");
        
        if (isPairedWithAudioPod)
        {
          audioPodFound = true;
          // set Silky volume to default.
          publishToSilkySetVolume(defaultSilkyVolume);
        }
        else
        {
          TeardownESPNow();
          ESPNowActive = false;
    
          M5.Lcd.println("   ESPNow\nDisabled");
          if (writeLogToSerial)
            USB_SERIAL.println("ESPNow Disabled");
        }
      }
      else
      {
        ESPNowActive = false;
        isPairedWithAudioPod = false;
      }
    }
    else
    { 
      // disconnect ESPNow;
      TeardownESPNow();
 
      ESPNowActive = false;
      isPairedWithAudioPod = false;

      M5.Lcd.println("ESPNow Disabled");
      
      if (writeLogToSerial)
        USB_SERIAL.println("ESPNow Disabled");
    }
    
    delay (500);
  
    M5.Lcd.fillScreen(TFT_BLACK);
  }
}

void toggleWiFiActive()
{
  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);

  if (ESPNowActive)
    toggleESPNowActive();

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
    M5.Lcd.setCursor(0, 10);
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

    M5.Lcd.fillScreen(TFT_ORANGE);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setRotation(1);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);

    M5.Lcd.printf(WiFi.status() == WL_CONNECTED ? "Wifi Enabled" : "No Connect");
    
    delay(2000);
  }

  M5.Lcd.fillScreen(TFT_BLACK);
}


void toggleOTAActive()
{
  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  M5.Lcd.setRotation(1);

  if (otaActiveListening)
  {
    asyncWebServer.end();
    M5.Lcd.println("OTA Disabled");
    otaActiveListening = false;
    delay (2000);
  }
  else
  {
    bool wifiToggled = false;
    if (WiFi.status() != WL_CONNECTED)
    {
      toggleWiFiActive();
      wifiToggled = true;

      M5.Lcd.fillScreen(TFT_ORANGE);
      M5.Lcd.setCursor(10, 10);
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
    }
    
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

      if (wifiToggled)
      {
        // Clear the QR Code from new wifi connection
        M5.Lcd.fillScreen(TFT_ORANGE);
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.setTextSize(3);
        M5.Lcd.setRotation(1);
        M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
        
        M5.Lcd.printf(" OTA & WiFi\n   Enabled");
      }
      else
      {
        M5.Lcd.printf("OTA Enabled");
      }
              
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

void toggleUptimeGlobalDisplay()
{
  enableGlobalUptimeDisplay = !enableGlobalUptimeDisplay;

  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(20, 10);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
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
  M5.Lcd.setCursor(20, 10);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
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
  
  if (writeLogToSerial)
    USB_SERIAL.println("ESPNow/Basic/Master Example");
    
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  // This is the mac address of the Master in Station Mode
  if (writeLogToSerial)
  {
    USB_SERIAL.print("STA MAC: "); USB_SERIAL.println(WiFi.macAddress());
    USB_SERIAL.print("STA CHANNEL "); USB_SERIAL.println(WiFi.channel());
  }

  // Init ESPNow with a fallback logic
  bool result =  InitESPNow();

  if (result)
  {
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnESPNowDataSent);
  }
  
  return result;
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

    if (writeLogToSerial)
      USB_SERIAL.println("Connecting to Qubitro...");

    if (!qubitro_mqttClient.connect(qubitro_host, qubitro_port))
    {
      if (writeLogToSerial)
      {
        USB_SERIAL.print("Connection failed. Error code: ");
        USB_SERIAL.println(qubitro_mqttClient.connectError());
        USB_SERIAL.println("Visit docs.qubitro.com or create a new issue on github.com/qubitro");
      }
      success = false;
    }
    else
    {
      if (writeLogToSerial)
        USB_SERIAL.println("Connected to Qubitro.");
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
        //USB_SERIAL.println(qubitro_payload);

        qubitro_mqttClient.poll();
        qubitro_mqttClient.beginMessage(qubitro_device_id);
        qubitro_mqttClient.print(qubitro_payload);
        int endMessageResult = qubitro_mqttClient.endMessage();
        if (endMessageResult == 1)
        {
          success = true;
          if (writeLogToSerial)
            USB_SERIAL.printf("Qubitro Client sent message %s\n", qubitro_payload);
        }
        else
        {
          if (writeLogToSerial)
            USB_SERIAL.printf("Qubitro Client failed to send message, EndMessage error: %d\n", endMessageResult);
        }
      }
      else
      {
        if (writeLogToSerial)
          USB_SERIAL.printf("Qubitro Client error status %d\n", qubitro_mqttClient.connectError());
      }
    }
    else
    {
      if (writeLogToSerial)
       USB_SERIAL.printf("Q No Wifi\n");
    }
  }
  else
  {
    if (writeLogToSerial)
      USB_SERIAL.printf("Q Not On\n");
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
    USB_SERIAL.println("Error connecting to SMTP, " + smtp.errorReason());
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
    USB_SERIAL.println("Error sending Email, " + smtp.errorReason());
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
    USB_SERIAL.println("Error connecting to SMTP, " + smtp.errorReason());
    return;
  }
  else
  {
    USB_SERIAL.println("Connected to SMTP Ok");
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
    USB_SERIAL.println("Error sending Email, " + smtp.errorReason());
  else
    USB_SERIAL.println("Error sending Email, " + smtp.errorReason());

}
#endif

// Init ESP Now with fallback
bool InitESPNow() 
{
  WiFi.disconnect();
  
  if (esp_now_init() == ESP_OK) 
  { 
    if (writeLogToSerial)
      USB_SERIAL.println("ESPNow Init Success");
    ESPNowActive = true;
  }
  else 
  {
    if (writeLogToSerial)
      USB_SERIAL.println("ESPNow Init Failed");
    // do nothing
    ESPNowActive = false;
  }
  
  return ESPNowActive;
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

// callback when data is sent from Master to Peer
void OnESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    ESPNowMessagesDelivered++;
  }
  else
  {
    ESPNowMessagesFailedToDeliver++;
  }

  /*
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  USB_SERIAL.print("Last Packet Sent to: "); USB_SERIAL.println(macStr);
  USB_SERIAL.print("Last Packet Send Status: ");

  USB_SERIAL.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

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

//  tb_display_print_String(ESPNowDiagBuffer);
*/
}

// Scan for peers in AP mode
bool ESPNowScanForPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix)
{
  bool peerFound = false;
  
  int8_t scanResults = WiFi.scanNetworks();
  // reset on each scan
 
  memset(&peer, 0, sizeof(peer));

  if (writeLogToSerial)
    USB_SERIAL.println("");

  if (scanResults == 0) 
  {    
    if (writeLogToSerial)
      USB_SERIAL.println("No WiFi devices in AP Mode found");

    peer.channel = ESPNOW_NO_PEER_CHANNEL_FLAG;
  } 
  else 
  {
    if (writeLogToSerial)
    {
      USB_SERIAL.print("Found "); USB_SERIAL.print(scanResults); USB_SERIAL.println(" devices ");
    }
    
    for (int i = 0; i < scanResults; ++i) 
    {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (writeLogToSerial && ESPNOW_PRINTSCANRESULTS) 
      {
        USB_SERIAL.print(i + 1);
        USB_SERIAL.print(": ");
        USB_SERIAL.print(SSID);
        USB_SERIAL.print(" (");
        USB_SERIAL.print(RSSI);
        USB_SERIAL.print(")");
        USB_SERIAL.println("");
      }
      
      delay(10);
      
      // Check if the current device starts with the peerSSIDPrefix
      if (SSID.indexOf(peerSSIDPrefix) == 0) 
      {
        if (writeLogToSerial)
        {
          // SSID of interest
          USB_SERIAL.println("Found a peer.");
          USB_SERIAL.print(i + 1); USB_SERIAL.print(": "); USB_SERIAL.print(SSID); USB_SERIAL.print(" ["); USB_SERIAL.print(BSSIDstr); USB_SERIAL.print("]"); USB_SERIAL.print(" ("); USB_SERIAL.print(RSSI); USB_SERIAL.print(")"); USB_SERIAL.println("");
        }
                
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) 
        {
          for (int ii = 0; ii < 6; ++ii ) 
          {
            peer.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        peer.channel = ESPNOW_CHANNEL; // pick a channel
        peer.encrypt = 0; // no encryption

        peerFound = true;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (peerFound) 
  {
    M5.Lcd.println("Peer Found");
    if (writeLogToSerial)
      USB_SERIAL.println("Peer Found, processing..");
  } 
  else 
  {
    M5.Lcd.println("Peer Not Found");
    if (writeLogToSerial)
      USB_SERIAL.println("Peer Not Found, trying again.");
  }
  
  // clean up ram
  WiFi.scanDelete();

  return peerFound;
}


bool pairWithPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix)
{
  int maxAttempts = 1;
  bool isPaired = false;
  while(maxAttempts-- && !isPaired)
  {
    bool result = ESPNowScanForPeer(peer,peerSSIDPrefix);

    // check if peer channel is defined
    if (result && peer.channel == ESPNOW_CHANNEL)
    { 
      isPaired = ESPNowManagePeer(peer);
      M5.Lcd.println("Pair ok");
    }
    else
    {
      peer.channel = ESPNOW_NO_PEER_CHANNEL_FLAG;
      M5.Lcd.println("Pair fail");
    }
  }

  delay(500);
  
  M5.Lcd.fillScreen(TFT_BLACK);
  
  return isPaired;
}

// Check if the peer is already paired with the master.
// If not, pair the peer with master
bool ESPNowManagePeer(esp_now_peer_info_t& peer)
{
  bool result = false;
  
  if (peer.channel == ESPNOW_CHANNEL) 
  {
    if (ESPNOW_DELETEBEFOREPAIR) 
    {
      ESPNowDeletePeer(peer);
    }

    if (writeLogToSerial)
      USB_SERIAL.print("Peer Status: ");
      
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer.peer_addr);
    
    if (exists) 
    {
      // Peer already paired.
      if (writeLogToSerial)
        USB_SERIAL.println("Already Paired");

      M5.Lcd.println("Already paired");
      result = true;
    } 
    else 
    {
      // Peer not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&peer);
      
      if (addStatus == ESP_OK) 
      {
        // Pair success
        if (writeLogToSerial)
          USB_SERIAL.println("Pair success");
        M5.Lcd.println("Pair success");
        result = true;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) 
      {
        // How did we get so far!!
        if (writeLogToSerial)
          USB_SERIAL.println("ESPNOW Not Init");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_ARG) 
      {
        if (writeLogToSerial)
            USB_SERIAL.println("Invalid Argument");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_FULL) 
      {
        if (writeLogToSerial)
            USB_SERIAL.println("Peer list full");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) 
      {
        if (writeLogToSerial)
          USB_SERIAL.println("Out of memory");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_EXIST) 
      {
        if (writeLogToSerial)
          USB_SERIAL.println("Peer Exists");
        result = true;
      } 
      else 
      {
        if (writeLogToSerial)
          USB_SERIAL.println("Not sure what happened");
        result = false;
      }
    }
  }
  else 
  {
    // No peer found to process
    if (writeLogToSerial)
      USB_SERIAL.println("No Peer found to process");
    
    M5.Lcd.println("No Peer found to process");
    result = false;
  }

  return result;
}

void ESPNowDeletePeer(esp_now_peer_info_t& peer) 
{
  if (peer.channel != ESPNOW_NO_PEER_CHANNEL_FLAG)
  {
    esp_err_t delStatus = esp_now_del_peer(peer.peer_addr);
    
    if (writeLogToSerial)
    {
      USB_SERIAL.print("Peer Delete Status: ");
      if (delStatus == ESP_OK) 
      {
        // Delete success
        USB_SERIAL.println("ESPNowDeletePeer::Success");
      } 
      else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) 
      {
        // How did we get so far!!
        USB_SERIAL.println("ESPNowDeletePeer::ESPNOW Not Init");
      } 
      else if (delStatus == ESP_ERR_ESPNOW_ARG) 
      {
        USB_SERIAL.println("ESPNowDeletePeer::Invalid Argument");
      } 
      else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) 
      {
        USB_SERIAL.println("ESPNowDeletePeer::Peer not found.");
      } 
      else 
      {
        USB_SERIAL.println("Not sure what happened");
      }
    }
  }
}
