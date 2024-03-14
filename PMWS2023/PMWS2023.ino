#include <BleKeyboard.h>
#include <M5Core2.h>
#include <Preferences.h>
#include <VL53L0X.h>
#include <Wire.h>

// Unit related constants
const int I2C_ADDR_VL53L0X = 0x29;
const int PIN_PORT_B_A = 33;
const int PIN_PORT_B_B = 32;
const int DIST_RANGE_MIN = 0;
const int DIST_RANGE_MAX = 2000;

const int GYRO_RANGE_MIN = -2000;
const int GYRO_RANGE_MAX = 2000;

const int ACC_RANGE_MIN = -80;
const int ACC_RANGE_MAX = 80;

const int UNIT_NONE = 0;
const int UNIT_DUAL_BUTTON = 1;

// Unit related variables
VL53L0X rangingSensor;
bool isDualButtonConnected = false;
bool isRangingSensorConnected = false;
bool isIMUSensorConnected = false;


int unitOnPortB = UNIT_NONE;
int distRangeMin = DIST_RANGE_MIN;
int distRangeMax = DIST_RANGE_MAX;

int gyroRangeMin_X = GYRO_RANGE_MIN;
int gyroRangeMax_X = GYRO_RANGE_MAX;

int gyroRangeMin_Y = GYRO_RANGE_MIN;
int gyroRangeMax_Y = GYRO_RANGE_MAX;

int gyroRangeMin_Z = GYRO_RANGE_MIN;
int gyroRangeMax_Z = GYRO_RANGE_MAX;


int accRangeMin = ACC_RANGE_MIN;
int accRangeMax = ACC_RANGE_MAX;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float gyroXBias = 0.0F;  // ジャイロZ軸のバイアス
float gyroYBias = 0.0F;  // ジャイロZ軸のバイアス
float gyroZBias = 0.0F;  // ジャイロZ軸のバイアス

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

// Screen related constants
const int LAYOUT_GYROZ_CH_TOP = 40;
const int LAYOUT_ANALOG_CH_TOP = 140;
const int LAYOUT_BUTTONS_CH_TOP = 160;
const int LAYOUT_LINE_HEIGHT = 20;
const int SCREEN_MAIN = 0;
const int SCREEN_PREFS_SELECT = 1;
const int SCREEN_PREFS_EDIT = 2;
const int PREFS_MENU_NUM_ITEMS = 9;
const int PREFS_MENU_INC_DEC_UNIT = 10;
const int PREFS_MENU_ACC_INC_DEC_UNIT = 1;

// Screen related variables
int currentMenuItem = 0;
int currentScreenMode = SCREEN_MAIN;
Preferences preferences;
char analogStatus[30];
char imuStatus[120];
char buttonsStatus1[30];
char buttonsStatus2[30];

// Protocol related constants
const byte KEYS_FOR_ANALOG_CH[] = {'`', '1', '2', '3', '4', '5',
                                   '6', '7', '8', '9', '0'
                                  };
const byte KEYS_FOR_BUTTON_CH[] = {'v', 'b'};


const byte KEYS_FOR_IMU_CH[] = {'m', 'n', ',', 'o', 'p', //gyro x
                                'u', 'i', 'h', 'j', 'k', //gyro y
                                'f', 'g', 'r', 't', 'y', //gyro z
                                'd', 'x', 'z', 'c', '[', //accel x
                                'q', 'w', 'e', 'a', 's', //accel y
                                'l', '-', '=', '.', '/'  //accel z
                               };


// ESP32 BLE Keyboard related variables
// Note: the device name should be within 15 characters;
// otherwise, macOS and iOS devices can't discover
// https://github.com/T-vK/ESP32-BLE-Keyboard/issues/51#issuecomment-733886764
# define BLEKEYBOARD_NAME "WMPS_"
BleKeyboard bleKeyboard(BLEKEYBOARD_NAME);
bool isConnected = false;
bool isSending = false;

bool wheeCtrl = false;

int BtnB_count = 0;


String bleKeyboard_Name = BLEKEYBOARD_NAME;

void setup() {
  // A workaround to avoid noise regarding Button A
  adc_power_acquire();

  M5.begin();
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLUE, WHITE);
  M5.Lcd.setTextSize(2);

  preferences.begin(BLEKEYBOARD_NAME, false);


  unitOnPortB = preferences.getInt("unitOnPortB", UNIT_NONE);
  distRangeMin = preferences.getInt("distRangeMin", DIST_RANGE_MIN);
  distRangeMax = preferences.getInt("distRangeMax", DIST_RANGE_MAX);

  gyroRangeMin_X = preferences.getInt("gyroRangeMin_X", GYRO_RANGE_MIN);
  gyroRangeMax_X = preferences.getInt("gyroRangeMax_X", GYRO_RANGE_MAX);

  gyroRangeMin_Y = preferences.getInt("gyroRangeMin_Y", GYRO_RANGE_MIN);
  gyroRangeMax_Y = preferences.getInt("gyroRangeMax_Y", GYRO_RANGE_MAX);

  gyroRangeMin_Z = preferences.getInt("gyroRangeMin_Z", GYRO_RANGE_MIN);
  gyroRangeMax_Z = preferences.getInt("gyroRangeMax_Z", GYRO_RANGE_MAX);


  accRangeMin = preferences.getInt("accRangeMin", ACC_RANGE_MIN);
  accRangeMax = preferences.getInt("accRangeMax", ACC_RANGE_MAX);


  updateFlagsRegardingPortB();

  pinMode(PIN_PORT_B_A, INPUT);
  pinMode(PIN_PORT_B_B, INPUT);

  Wire.begin();

  rangingSensor.setTimeout(500);
  if (rangingSensor.init()) {
    isRangingSensorConnected = true;
    rangingSensor.setMeasurementTimingBudget(20000);
  }

  M5.IMU.Init();

  // ジャイロスコープのバイアス（Z軸）を計算
  float sumGyroX = 0.0F;
  float sumGyroY = 0.0F;
  float sumGyroZ = 0.0F;



  int sampleCount = 500;
  for (int i = 0; i < sampleCount; i++) {
    float x, y, z;
    M5.IMU.getGyroData(&x, &y, &z);
    sumGyroX += x;
    sumGyroY += y;
    sumGyroZ += z;

    delay(2);  // サンプル間の短い遅延
  }
  gyroXBias = sumGyroX / sampleCount;  // 平均をバイアスとして使用
  gyroYBias = sumGyroY / sampleCount;  // 平均をバイアスとして使用
  gyroZBias = sumGyroZ / sampleCount;  // 平均をバイアスとして使用


  bleKeyboard.begin();

  M5.Lcd.setCursor(0, 0);
  //M5.Lcd.print("Bluetooth: Not connected");
  M5.Lcd.print(bleKeyboard_Name);
  M5.Lcd.print(": Not connected");


  drawButtons(currentScreenMode);
  Serial.begin(9600);
}

void loop() {
  const unsigned long LOOP_INTERVAL = 25;
  unsigned long start = millis();

  M5.update();
  handleButtons();

  isConnected = bleKeyboard.isConnected();
  bool requestToSend = isConnected && isSending;


  handleIMUSensor(requestToSend);


  if (currentScreenMode == SCREEN_MAIN) {
    drawMainScreen();
  }

  unsigned long now = millis();
  unsigned long elapsed = now - start;


  if (elapsed < LOOP_INTERVAL) {
    delay(LOOP_INTERVAL - elapsed);
  }
}

void handleButtons() {
  if (currentScreenMode != SCREEN_MAIN) {
    M5.Lcd.setCursor(0, 0 + LAYOUT_LINE_HEIGHT * currentMenuItem);
    M5.Lcd.print(">");
  }

  if (M5.BtnA.wasPressed()) {
    switch (currentScreenMode) {
      case SCREEN_MAIN:
        currentScreenMode = SCREEN_PREFS_SELECT;
        M5.Lcd.clear(TFT_WHITE);
        drawButtons(currentScreenMode);
        break;
      case SCREEN_PREFS_SELECT:
        currentScreenMode = SCREEN_MAIN;
        M5.Lcd.clear(TFT_WHITE);
        drawButtons(currentScreenMode);
        break;
      case SCREEN_PREFS_EDIT:
        switch (currentMenuItem) {
          case 0:
            unitOnPortB = unitOnPortB - 1;
            if (unitOnPortB < 0) {
              unitOnPortB = UNIT_DUAL_BUTTON;
            }
            preferences.putInt("unitOnPortB", unitOnPortB);
            updateFlagsRegardingPortB();
            break;


          case 1:
            gyroRangeMin_X = constrain((gyroRangeMin_X + PREFS_MENU_INC_DEC_UNIT),
                                       GYRO_RANGE_MIN,
                                       gyroRangeMax_X - PREFS_MENU_INC_DEC_UNIT);
            preferences.putInt("gyroRangeMin_X", gyroRangeMin_X);
            break;
          case 2:
            gyroRangeMax_X = constrain((gyroRangeMax_X - PREFS_MENU_INC_DEC_UNIT),
                                       gyroRangeMin_X + PREFS_MENU_INC_DEC_UNIT,
                                       GYRO_RANGE_MAX);
            preferences.putInt("gyroRangeMax_X", gyroRangeMax_X);
            break;


          case 3:
            gyroRangeMin_Y = constrain((gyroRangeMin_Y + PREFS_MENU_INC_DEC_UNIT),
                                       GYRO_RANGE_MIN,
                                       gyroRangeMax_Y - PREFS_MENU_INC_DEC_UNIT);
            preferences.putInt("gyroRangeMin_Y", gyroRangeMin_Y);
            break;
          case 4:
            gyroRangeMax_Y = constrain((gyroRangeMax_Y - PREFS_MENU_INC_DEC_UNIT),
                                       gyroRangeMin_Y + PREFS_MENU_INC_DEC_UNIT,
                                       GYRO_RANGE_MAX);
            preferences.putInt("gyroRangeMax_Y", gyroRangeMax_Y);
            break;



          case 5:
            gyroRangeMin_Z = constrain((gyroRangeMin_Z + PREFS_MENU_INC_DEC_UNIT),
                                       GYRO_RANGE_MIN,
                                       gyroRangeMax_Z - PREFS_MENU_INC_DEC_UNIT);
            preferences.putInt("gyroRangeMin_Z", gyroRangeMin_Z);
            break;
          case 6:
            gyroRangeMax_Z = constrain((gyroRangeMax_Z - PREFS_MENU_INC_DEC_UNIT),
                                       gyroRangeMin_Z + PREFS_MENU_INC_DEC_UNIT,
                                       GYRO_RANGE_MAX);
            preferences.putInt("gyroRangeMax_Z", gyroRangeMax_Z);
            break;

          case 7:
            accRangeMin = constrain((accRangeMin + PREFS_MENU_ACC_INC_DEC_UNIT),
                                    ACC_RANGE_MIN,
                                    accRangeMax - PREFS_MENU_ACC_INC_DEC_UNIT);
            preferences.putInt("accRangeMin", accRangeMin);
            break;
          case 8:
            accRangeMax = constrain((accRangeMax - PREFS_MENU_ACC_INC_DEC_UNIT),
                                    accRangeMin + PREFS_MENU_ACC_INC_DEC_UNIT,
                                    ACC_RANGE_MAX);
            preferences.putInt("accRangeMax", accRangeMax);
            break;

          default:
            break;
        }
        break;
      default:
        break;
    }
  }

  if (M5.BtnC.wasPressed()) {
    switch (currentScreenMode) {
      case SCREEN_MAIN:
        if (isSending) {
          isSending = false;
          drawButtons(currentScreenMode);
        } else {
          isSending = true;
          drawButtons(currentScreenMode);
        }
        break;
      case SCREEN_PREFS_SELECT:
        M5.Lcd.setCursor(0, 0 + LAYOUT_LINE_HEIGHT * currentMenuItem);
        M5.Lcd.print(" ");
        currentMenuItem = (currentMenuItem + 1) % PREFS_MENU_NUM_ITEMS;
        M5.Lcd.setCursor(0, 0 + LAYOUT_LINE_HEIGHT * currentMenuItem);
        M5.Lcd.print(">");
        drawButtons(currentScreenMode);
        break;

      case SCREEN_PREFS_EDIT:
        switch (currentMenuItem) {
          case 0:
            unitOnPortB = unitOnPortB + 1;
            if (unitOnPortB > UNIT_DUAL_BUTTON) {
              unitOnPortB = UNIT_NONE;
            }
            preferences.putInt("unitOnPortB", unitOnPortB);
            updateFlagsRegardingPortB();
            break;

          case 1:
            gyroRangeMin_X = constrain((gyroRangeMin_X - PREFS_MENU_INC_DEC_UNIT),
                                       GYRO_RANGE_MIN,
                                       gyroRangeMax_X - PREFS_MENU_INC_DEC_UNIT);
            preferences.putInt("gyroRangeMin_X", gyroRangeMin_X);
            break;
          case 2:
            gyroRangeMax_X = constrain((gyroRangeMax_X + PREFS_MENU_INC_DEC_UNIT),
                                       gyroRangeMin_X + PREFS_MENU_INC_DEC_UNIT,
                                       GYRO_RANGE_MAX);
            preferences.putInt("gyroRangeMax_X", gyroRangeMax_X);
            break;



          case 3:
            gyroRangeMin_Y = constrain((gyroRangeMin_Y - PREFS_MENU_INC_DEC_UNIT),
                                       GYRO_RANGE_MIN,
                                       gyroRangeMax_Y - PREFS_MENU_INC_DEC_UNIT);
            preferences.putInt("gyroRangeMin_Y", gyroRangeMin_Y);
            break;
          case 4:
            gyroRangeMax_Y = constrain((gyroRangeMax_Y + PREFS_MENU_INC_DEC_UNIT),
                                       gyroRangeMin_Y + PREFS_MENU_INC_DEC_UNIT,
                                       GYRO_RANGE_MAX);
            preferences.putInt("gyroRangeMax_Y", gyroRangeMax_Y);
            break;


          case 5:
            gyroRangeMin_Z = constrain((gyroRangeMin_Z - PREFS_MENU_INC_DEC_UNIT),
                                       GYRO_RANGE_MIN,
                                       gyroRangeMax_Z - PREFS_MENU_INC_DEC_UNIT);
            preferences.putInt("gyroRangeMin_Z", gyroRangeMin_Z);
            break;
          case 6:
            gyroRangeMax_Z = constrain((gyroRangeMax_Z + PREFS_MENU_INC_DEC_UNIT),
                                       gyroRangeMin_Z + PREFS_MENU_INC_DEC_UNIT,
                                       GYRO_RANGE_MAX);
            preferences.putInt("gyroRangeMax_Z", gyroRangeMax_Z);
            break;

          case 7:
            accRangeMin = constrain((accRangeMin - PREFS_MENU_ACC_INC_DEC_UNIT),
                                    ACC_RANGE_MIN,
                                    accRangeMax - PREFS_MENU_ACC_INC_DEC_UNIT);
            preferences.putInt("accRangeMin", accRangeMin);
            break;
          case 8:
            accRangeMax = constrain((accRangeMax + PREFS_MENU_ACC_INC_DEC_UNIT),
                                    accRangeMin + PREFS_MENU_ACC_INC_DEC_UNIT,
                                    ACC_RANGE_MAX);
            preferences.putInt("accRangeMax", accRangeMax);
            break;

          default:
            break;
        }
        break;
      default:
        break;
    }
  }

  if (M5.BtnA.pressedFor(500)) {
    if (currentScreenMode == SCREEN_PREFS_EDIT) {
      switch (currentMenuItem) {
        case 1:
          gyroRangeMin_X = constrain((gyroRangeMin_X + PREFS_MENU_INC_DEC_UNIT),
                                     GYRO_RANGE_MIN,
                                     gyroRangeMax_X - PREFS_MENU_INC_DEC_UNIT);
          break;
        case 2:
          gyroRangeMax_X = constrain((gyroRangeMax_X - PREFS_MENU_INC_DEC_UNIT),
                                     gyroRangeMin_X + PREFS_MENU_INC_DEC_UNIT,
                                     GYRO_RANGE_MAX);
          break;


        case 3:
          gyroRangeMin_Y = constrain((gyroRangeMin_Y + PREFS_MENU_INC_DEC_UNIT),
                                     GYRO_RANGE_MIN,
                                     gyroRangeMax_Y - PREFS_MENU_INC_DEC_UNIT);
          break;
        case 4:
          gyroRangeMax_Y = constrain((gyroRangeMax_Y - PREFS_MENU_INC_DEC_UNIT),
                                     gyroRangeMin_Y + PREFS_MENU_INC_DEC_UNIT,
                                     GYRO_RANGE_MAX);
          break;


        case 5:
          gyroRangeMin_Z = constrain((gyroRangeMin_Z + PREFS_MENU_INC_DEC_UNIT),
                                     GYRO_RANGE_MIN,
                                     gyroRangeMax_Z - PREFS_MENU_INC_DEC_UNIT);
          break;
        case 6:
          gyroRangeMax_Z = constrain((gyroRangeMax_Z - PREFS_MENU_INC_DEC_UNIT),
                                     gyroRangeMin_Z + PREFS_MENU_INC_DEC_UNIT,
                                     GYRO_RANGE_MAX);
          break;

        case 7:
          accRangeMin = constrain((accRangeMin + PREFS_MENU_ACC_INC_DEC_UNIT),
                                  ACC_RANGE_MIN,
                                  accRangeMax - PREFS_MENU_ACC_INC_DEC_UNIT);
          preferences.putInt("accRangeMin", accRangeMin);
          break;
        case 8:
          accRangeMax = constrain((accRangeMax - PREFS_MENU_ACC_INC_DEC_UNIT),
                                  accRangeMin + PREFS_MENU_ACC_INC_DEC_UNIT,
                                  ACC_RANGE_MAX);
          preferences.putInt("accRangeMax", accRangeMax);
          break;
        default:
          break;
      }
    }
  }

  if (M5.BtnC.pressedFor(500)) {
    if (currentScreenMode == SCREEN_PREFS_EDIT) {
      switch (currentMenuItem) {

        case 1:
          gyroRangeMin_X = constrain((gyroRangeMin_X - PREFS_MENU_INC_DEC_UNIT),
                                     GYRO_RANGE_MIN,
                                     gyroRangeMax_X - PREFS_MENU_INC_DEC_UNIT);
          break;
        case 2:
          gyroRangeMax_X = constrain((gyroRangeMax_X + PREFS_MENU_INC_DEC_UNIT),
                                     gyroRangeMin_X + PREFS_MENU_INC_DEC_UNIT,
                                     GYRO_RANGE_MAX);

        case 3:
          gyroRangeMin_Y = constrain((gyroRangeMin_Y - PREFS_MENU_INC_DEC_UNIT),
                                     GYRO_RANGE_MIN,
                                     gyroRangeMax_Y - PREFS_MENU_INC_DEC_UNIT);
          break;
        case 4:
          gyroRangeMax_Y = constrain((gyroRangeMax_Y + PREFS_MENU_INC_DEC_UNIT),
                                     gyroRangeMin_Y + PREFS_MENU_INC_DEC_UNIT,
                                     GYRO_RANGE_MAX);
          break;

        case 5:
          gyroRangeMin_Z = constrain((gyroRangeMin_Z - PREFS_MENU_INC_DEC_UNIT),
                                     GYRO_RANGE_MIN,
                                     gyroRangeMax_Z - PREFS_MENU_INC_DEC_UNIT);
          break;
        case 6:
          gyroRangeMax_Z = constrain((gyroRangeMax_Z + PREFS_MENU_INC_DEC_UNIT),
                                     gyroRangeMin_Z + PREFS_MENU_INC_DEC_UNIT,
                                     GYRO_RANGE_MAX);
          break;

        case 7:
          accRangeMin = constrain((accRangeMin - PREFS_MENU_ACC_INC_DEC_UNIT),
                                  ACC_RANGE_MIN,
                                  accRangeMax - PREFS_MENU_ACC_INC_DEC_UNIT);
          preferences.putInt("accRangeMin", accRangeMin);
          break;
        case 8:
          accRangeMax = constrain((accRangeMax + PREFS_MENU_ACC_INC_DEC_UNIT),
                                  accRangeMin + PREFS_MENU_ACC_INC_DEC_UNIT,
                                  ACC_RANGE_MAX);
          preferences.putInt("accRangeMax", accRangeMax);
          break;

        default:
          break;
      }
    }
  }

  if (M5.BtnB.wasPressed()) {
    switch (currentScreenMode) {

      case SCREEN_MAIN:

        break;

      case SCREEN_PREFS_SELECT:

        currentScreenMode = SCREEN_PREFS_EDIT;

        break;

      case SCREEN_PREFS_EDIT:
        currentScreenMode = SCREEN_PREFS_SELECT;
        break;
      default:
        break;
    }

    drawButtons(currentScreenMode);
  }

  if (currentScreenMode != SCREEN_MAIN) {
    drawPreferencesScreen();
  }
}

void updateFlagsRegardingPortB() {
  switch (unitOnPortB) {
    case UNIT_NONE:
      wheeCtrl = false;
      break;
    case UNIT_DUAL_BUTTON:
      wheeCtrl = true;
      break;

    default:
      break;
  }
}

void drawMainScreen() {
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print(bleKeyboard_Name);
  M5.Lcd.printf(": %s",
                isConnected ? "Connected    " : "Disconnected ");
  //imu
  M5.Lcd.setCursor(0, LAYOUT_GYROZ_CH_TOP);
  M5.Lcd.print(imuStatus);

  M5.Lcd.setCursor(0, LAYOUT_ANALOG_CH_TOP);
  M5.Lcd.print(analogStatus);

  M5.Lcd.setCursor(0, LAYOUT_BUTTONS_CH_TOP);
  M5.Lcd.print("Whill Control:");
  if (wheeCtrl) {
    M5.Lcd.print("Yes");
  } else {
    M5.Lcd.print("No");
  }
}

void drawPreferencesScreen() {
  M5.Lcd.setCursor(20, 0 + LAYOUT_LINE_HEIGHT * 0);
  switch (unitOnPortB) {
    case UNIT_NONE:
      M5.Lcd.print("Whill Control: NO ");
      break;
    case UNIT_DUAL_BUTTON:
      M5.Lcd.print("Whill Control: YES");
      break;
    default:
      break;
  }

  M5.Lcd.setCursor(20, 0 + LAYOUT_LINE_HEIGHT * 1);
  M5.Lcd.printf("gyroX Range Min: %5d", gyroRangeMin_X);
  M5.Lcd.setCursor(20, 0 + LAYOUT_LINE_HEIGHT * 2);
  M5.Lcd.printf("           Max: %5d", gyroRangeMax_X);

  M5.Lcd.setCursor(20, 0 + LAYOUT_LINE_HEIGHT * 3);
  M5.Lcd.printf("gyroY Range Min: %5d", gyroRangeMin_Y);
  M5.Lcd.setCursor(20, 0 + LAYOUT_LINE_HEIGHT * 4);
  M5.Lcd.printf("           Max: %5d", gyroRangeMax_Y);


  M5.Lcd.setCursor(20, 0 + LAYOUT_LINE_HEIGHT * 5);
  M5.Lcd.printf("gyroZ Range Min: %5d", gyroRangeMin_Z);
  M5.Lcd.setCursor(20, 0 + LAYOUT_LINE_HEIGHT * 6);
  M5.Lcd.printf("           Max: %5d", gyroRangeMax_Z);

  M5.Lcd.setCursor(20, 0 + LAYOUT_LINE_HEIGHT * 7);
  M5.Lcd.printf("acc Range  Min: %5d", accRangeMin);
  M5.Lcd.setCursor(20, 0 + LAYOUT_LINE_HEIGHT * 8);
  M5.Lcd.printf("           Max: %5d", accRangeMax);
}

void handleDualButton(bool updateRequested) {
  const int KEY_ID_RED_BUTTON = 0;
  const int KEY_ID_BLUE_BUTTON = 1;

  static bool wasRedButtonPressed = false;
  static bool wasBlueButtonPressed = false;

  bool isRedButtonPressed = digitalRead(PIN_PORT_B_B) == LOW;
  bool isBlueButtonPressed = digitalRead(PIN_PORT_B_A) == LOW;

  sprintf(buttonsStatus1, "Red:%d  Blue:%d", isRedButtonPressed,
          isBlueButtonPressed);

  if (updateRequested) {
    if (!wasRedButtonPressed && isRedButtonPressed) {
      bleKeyboard.press(KEYS_FOR_BUTTON_CH[KEY_ID_RED_BUTTON]);
    } else if (wasRedButtonPressed && !isRedButtonPressed) {
      bleKeyboard.release(KEYS_FOR_BUTTON_CH[KEY_ID_RED_BUTTON]);
    }

    if (!wasBlueButtonPressed && isBlueButtonPressed) {
      bleKeyboard.press(KEYS_FOR_BUTTON_CH[KEY_ID_BLUE_BUTTON]);
    } else if (wasBlueButtonPressed && !isBlueButtonPressed) {
      bleKeyboard.release(KEYS_FOR_BUTTON_CH[KEY_ID_BLUE_BUTTON]);
    }
  }

  wasRedButtonPressed = isRedButtonPressed;
  wasBlueButtonPressed = isBlueButtonPressed;
}

void handleRangingSensor(bool updateRequested) {
  static int lastValue = -1;

  int range = constrain(rangingSensor.readRangeSingleMillimeters(),
                        distRangeMin, distRangeMax);

  if (rangingSensor.readRangeSingleMillimeters() == 8191)
  {
    range = distRangeMax;
  }
  // convert to 11 steps
  int currentValue = map(range, distRangeMin, distRangeMax, 0, 10);

  Serial.println(rangingSensor.readRangeSingleMillimeters());

  sprintf(analogStatus, "ANALOG:%2d (%4d mm)", currentValue, range);

  if (lastValue != currentValue) {

    if (updateRequested) {
      bleKeyboard.write(KEYS_FOR_ANALOG_CH[currentValue]);
    }
    lastValue = currentValue;
  }

}

int currentValue_gyroX;
void handleIMUSensor(bool updateRequested) {
  static int lastValue_gyroX = 0;
  static int lastValue_gyroY = 0;
  static int lastValue_gyroZ = 0;

  static int lastValue_accX = 0;
  static int lastValue_accY = 0;
  static int lastValue_accZ = 0;

  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);

  gyroX -= gyroXBias;  // バイアス補正を適用
  gyroY -= gyroYBias;  // バイアス補正を適用
  gyroZ -= gyroZBias;  // バイアス補正を適用


  int gyroX_data = constrain(gyroX, gyroRangeMin_X, gyroRangeMax_X);
  int gyroY_data = constrain(gyroY, gyroRangeMin_Y, gyroRangeMax_Y);
  int gyroZ_data = constrain(gyroZ, gyroRangeMin_Z, gyroRangeMax_Z);

  int accX_data = constrain(accX * 10, accRangeMin, accRangeMax);
  int accY_data = constrain(accY * 10, accRangeMin, accRangeMax);
  int accZ_data = constrain(accZ * 10, accRangeMin, accRangeMax);


  // convert to 5 steps
  int currentValue_gyroX = int(round(preciseMap(gyroX_data, gyroRangeMin_X, gyroRangeMax_X, -2, 2)));
  int currentValue_gyroY = int(round(preciseMap(gyroY_data, gyroRangeMin_Y, gyroRangeMax_Y, -2, 2)));
  int currentValue_gyroZ = int(round(preciseMap(gyroZ_data, gyroRangeMin_Z, gyroRangeMax_Z, -2, 2)));

  int currentValue_accX = int(round(preciseMap(accX_data, accRangeMin, accRangeMax, -2, 2)));
  int currentValue_accY = int(round(preciseMap(accY_data, accRangeMin, accRangeMax, -2, 2)));
  int currentValue_accZ = int(round(preciseMap(accZ_data, accRangeMin, accRangeMax, -2, 2)));

  sprintf(imuStatus, "GryoX:%2d (%4d o/s)\nGryoY:%2d (%4d o/s)\nGryoZ:%2d (%4d o/s)\nAccX:%2d (%4d G/10)\nAccY:%2d (%4d G/10)\nAccZ:%2d (%4d G/10)", currentValue_gyroX, gyroX_data, currentValue_gyroY, gyroY_data, currentValue_gyroZ, gyroZ_data, currentValue_accX, accX_data, currentValue_accY, accY_data, currentValue_accZ, accZ_data);

  if (lastValue_gyroX != currentValue_gyroX) {
    if (updateRequested) {
      if (wheeCtrl == false) {
        bleKeyboard.write(KEYS_FOR_IMU_CH[currentValue_gyroX + 2]);//+2: [-2,2] -> [0,4] move cursor to gyroX
      }
    }
    lastValue_gyroX = currentValue_gyroX;
  }

  if (lastValue_gyroY != currentValue_gyroY) {
    if (updateRequested) {
      if (wheeCtrl == false) {
        bleKeyboard.write(KEYS_FOR_IMU_CH[currentValue_gyroY + 2 + 5]);//+2: [-2,2] -> [0,4], +5: move cursor to gyroY
      }

      if (wheeCtrl)
      {
        if (currentValue_gyroY + 7 == 5)
        {
          bleKeyboard.release(KEY_LEFT_ARROW);
          bleKeyboard.press(KEY_RIGHT_ARROW);
        }
        else if (currentValue_gyroY + 7 == 9)
        {
          bleKeyboard.release(KEY_RIGHT_ARROW);
          bleKeyboard.press(KEY_LEFT_ARROW);
        }
        else
        {
          bleKeyboard.release(KEY_LEFT_ARROW);
          bleKeyboard.release(KEY_RIGHT_ARROW);
        }
      }
    }

    lastValue_gyroY = currentValue_gyroY;
  }


  if (lastValue_gyroZ != currentValue_gyroZ) {
    if (updateRequested) {
      if (wheeCtrl == false) {
        bleKeyboard.write(KEYS_FOR_IMU_CH[currentValue_gyroZ + 2 + 10]);//+2: [-2,2] -> [0,4], +10: move cursor to gyroZ
      }

            if (wheeCtrl)
            {
              if (currentValue_gyroZ + 12 == 10 )
              {
                bleKeyboard.release(KEY_UP_ARROW);
                bleKeyboard.press(KEY_DOWN_ARROW);
      
              }
              else if ( currentValue_gyroZ + 12 == 14)
              {
                bleKeyboard.release(KEY_DOWN_ARROW);
                bleKeyboard.press(KEY_UP_ARROW);
              }
              else {
                bleKeyboard.release(KEY_UP_ARROW);
                bleKeyboard.release(KEY_DOWN_ARROW);
              }
            }
    }


    lastValue_gyroZ = currentValue_gyroZ;
  }




  if (lastValue_accX != currentValue_accX) {
    if (updateRequested) {
      if (wheeCtrl == false) {
        bleKeyboard.write(KEYS_FOR_IMU_CH[currentValue_accX + 2 + 15]); //+2: [-2,2] -> [0,4], +15: move cursor to accX
      }
    }
    lastValue_accX = currentValue_accX;
  }

  if (lastValue_accY != currentValue_accY) {
    if (updateRequested) {
      if (wheeCtrl == false) {
        bleKeyboard.write(KEYS_FOR_IMU_CH[currentValue_accY + 2 + 20]);//+2: [-2,2] -> [0,4], +20: move cursor to gyroY
      }
    }
    lastValue_accY = currentValue_accY;
  }


  if (lastValue_accZ != currentValue_accZ) {
    if (updateRequested) {
      if (wheeCtrl == false) {
        bleKeyboard.write(KEYS_FOR_IMU_CH[currentValue_accZ + 2 + 25]);//+2: [-2,2] -> [0,4], +10: move cursor to gyroZ
      }
    }
    lastValue_accZ = currentValue_accZ;
  }



}

void drawButtons(int currentScreenMode) {
  const int LAYOUT_BTN_A_CENTER = 64;
  const int LAYOUT_BTN_B_CENTER = 160;
  const int LAYOUT_BTN_C_CENTER = 256;

  switch (currentScreenMode) {
    case SCREEN_MAIN:
      if (!isSending) {
        drawButton(LAYOUT_BTN_A_CENTER, "Setup");


        drawButton(LAYOUT_BTN_C_CENTER, "Send");
      } else {
        drawButton(LAYOUT_BTN_A_CENTER, "Setup");

        drawButton(LAYOUT_BTN_C_CENTER, "Stop");
      }
      break;
    case SCREEN_PREFS_SELECT:
      drawButton(LAYOUT_BTN_A_CENTER, "Exit");
      drawButton(LAYOUT_BTN_B_CENTER, "Enter");
      drawButton(LAYOUT_BTN_C_CENTER, "Next");
      break;
    case SCREEN_PREFS_EDIT:
      drawButton(LAYOUT_BTN_A_CENTER, "-");
      drawButton(LAYOUT_BTN_B_CENTER, "Done");
      drawButton(LAYOUT_BTN_C_CENTER, "+");
      break;
    default:
      break;
  }
}

void drawButton(int centerX, const String & title) {
  const int BUTTON_WIDTH = 72;
  const int BUTTON_HEIGHT = 24;

  M5.Lcd.setTextSize(2);

  int fontHeight = M5.Lcd.fontHeight();
  int rectLeft = centerX - BUTTON_WIDTH / 2;
  int rectTop = M5.Lcd.height() - BUTTON_HEIGHT;
  int rectWidth = BUTTON_WIDTH;
  int rectHeight = BUTTON_HEIGHT;
  int coordinateY = rectTop + (rectHeight - fontHeight) / 2;

  M5.Lcd.fillRect(rectLeft, rectTop, rectWidth, rectHeight, TFT_WHITE);
  M5.Lcd.drawRect(rectLeft, rectTop, rectWidth, rectHeight, TFT_BLUE);
  M5.Lcd.drawCentreString(title, centerX, coordinateY, 1);
}


//map 関数
float preciseMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
