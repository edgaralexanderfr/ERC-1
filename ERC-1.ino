#include "MemoryFree.h"
#include "NewPing.h"
#include "NewTone.h"

const PROGMEM uint8_t PACKET_SIZE                                  = 6;
const PROGMEM uint8_t PACKET_DELIMITER                             = 124;
const PROGMEM uint8_t IN_SET_DIRECTION_AND_ACCELERATION            = 2;
const PROGMEM uint8_t IN_SET_FORWARD                               = 3;
const PROGMEM uint8_t IN_SET_NEUTRAL                               = 4;
const PROGMEM uint8_t IN_SET_REVERSE                               = 5;
const PROGMEM uint8_t IN_START_BRAKING                             = 6;
const PROGMEM uint8_t IN_STOP_BRAKING                              = 7;
const PROGMEM uint8_t IN_ACCELERATE                                = 8;
const PROGMEM uint8_t IN_DECELERATE                                = 9;
const PROGMEM uint8_t IN_SET_MOTORS_LEVEL_FACTOR                   = 10;
const PROGMEM uint8_t IN_SET_DIRECTION_FOR_ROTATION                = 11;
const PROGMEM uint8_t IN_TOGGLE_LEFT_BLINKER                       = 15;
const PROGMEM uint8_t IN_TOGGLE_HEADLIGHTS                         = 16;
const PROGMEM uint8_t IN_START_HORN                                = 17;
const PROGMEM uint8_t IN_STOP_HORN                                 = 18;
const PROGMEM uint8_t IN_TOGGLE_SIREN                              = 19;
const PROGMEM uint8_t IN_TOGGLE_EMERGENCY_LIGHTS                   = 20;
const PROGMEM uint8_t IN_TOGGLE_RIGHT_BLINKER                      = 21;
const PROGMEM uint8_t OUT_LOG_WRITE                                = 0;
const PROGMEM uint8_t OUT_LOG_WRITE_FREE_MEMORY                    = 1;
const PROGMEM uint8_t OUT_UPDATE_COLLISION_SECTORS                 = 2;
const PROGMEM uint8_t RIGHT_LIGHTS_PIN                             = 2;
const PROGMEM uint8_t LEFT_MOTORS_DIRECTION_PIN                    = 3;
const PROGMEM uint8_t RIGHT_MOTORS_DIRECTION_PIN                   = 4;
const PROGMEM uint8_t LEFT_MOTORS_PIN                              = 5;
const PROGMEM uint8_t RIGHT_MOTORS_PIN                             = 6;
const PROGMEM uint8_t LEFT_LIGHTS_PIN                              = 7;
const PROGMEM uint8_t BEEP_PIN                                     = 10;
const PROGMEM uint8_t HEADLIGHTS_PIN                               = 11;
const PROGMEM uint8_t NORTH_WEST_USS_TRIG_PIN                      = A2;
const PROGMEM uint8_t NORTH_WEST_USS_ECHO_PIN                      = A3;
const PROGMEM uint8_t NORTH_EAST_USS_TRIG_PIN                      = A0;
const PROGMEM uint8_t NORTH_EAST_USS_ECHO_PIN                      = A1;
const PROGMEM uint8_t SOUTH_WEST_USS_TRIG_PIN                      = A4;
const PROGMEM uint8_t SOUTH_WEST_USS_ECHO_PIN                      = A5;
const PROGMEM uint8_t SOUTH_EAST_USS_TRIG_PIN                      = 13;
const PROGMEM uint8_t SOUTH_EAST_USS_ECHO_PIN                      = 12;
const PROGMEM uint8_t CLOCK_TIME                                   = 20;
const PROGMEM uint8_t HEADLIGHTS_LEVEL_FACTOR                      = 30;
const PROGMEM uint8_t SIREN_LEVEL_FACTOR                           = 10;
const PROGMEM uint8_t BACK_UP_BEEP_MAX_COUNT                       = 16;
const PROGMEM uint8_t LIGHTS_MAX_COUNT                             = 12;
const PROGMEM uint8_t COLLISION_SECTORS_DISTANCES_REPORT_MAX_COUNT = 33;
const PROGMEM uint16_t SERIAL_PORT_BPS                             = 9600;
const PROGMEM uint16_t HORN_FREQUENCY                              = 405;
const PROGMEM uint16_t SIREN_MIN_FREQUENCY                         = 635;
const PROGMEM uint16_t SIREN_MAX_FREQUENCY                         = 912;
const PROGMEM uint16_t BACK_UP_BEEP_FREQUENCY                      = 1000;
const PROGMEM uint16_t USS_MAX_DISTANCE                            = 400;

bool braking                                                       = 0;
bool digitalAcceleration                                           = 0;
bool headlightsOn                                                  = 0;
bool hornOn                                                        = 0;
bool sirenOn                                                       = 0;
bool backUpBeep                                                    = 0;
bool emergencyLightsOn                                             = 0;
bool leftBlinkerOn                                                 = 0;
bool rightBlinkerOn                                                = 0;
bool leftLightsOn                                                  = 0;
bool rightLightsOn                                                 = 0;
bool leftLightsBlinking                                            = 0;
bool rightLightsBlinking                                           = 0;
int8_t gear                                                        = 1;
int8_t sirenLevelFactor                                            = SIREN_LEVEL_FACTOR;
uint8_t direction                                                  = 127;
uint8_t directionForRotation                                       = 100;
uint8_t acceleration                                               = 0;
uint8_t motorsLevelFactor                                          = 32;
uint8_t headlightsPulse                                            = 0;
uint8_t headlightsFinalPulse                                       = 0;
uint8_t backUpBeepCount                                            = 0;
uint8_t lightsCount                                                = 0;
uint8_t collisionSectorsDistancesReportCount                       = 0;
int16_t leftMotorsPulse                                            = 0;
int16_t rightMotorsPulse                                           = 0;
int16_t leftMotorsFinalPulse                                       = 0;
int16_t rightMotorsFinalPulse                                      = 0;
uint16_t sirenFrequency                                            = SIREN_MIN_FREQUENCY;
uint16_t collisionSectorsMaxDistance                               = 100;
uint16_t northWestCollisionSectorDistance                          = collisionSectorsMaxDistance;
uint16_t northEastCollisionSectorDistance                          = collisionSectorsMaxDistance;
uint16_t southWestCollisionSectorDistance                          = collisionSectorsMaxDistance;
uint16_t southEastCollisionSectorDistance                          = collisionSectorsMaxDistance;

NewPing northWestUSS(NORTH_WEST_USS_TRIG_PIN, NORTH_WEST_USS_ECHO_PIN, USS_MAX_DISTANCE);
NewPing northEastUSS(NORTH_EAST_USS_TRIG_PIN, NORTH_EAST_USS_ECHO_PIN, USS_MAX_DISTANCE);
NewPing southWestUSS(SOUTH_WEST_USS_TRIG_PIN, SOUTH_WEST_USS_ECHO_PIN, USS_MAX_DISTANCE);
NewPing southEastUSS(SOUTH_EAST_USS_TRIG_PIN, SOUTH_EAST_USS_ECHO_PIN, USS_MAX_DISTANCE);

bool readPacket (int8_t * name, int8_t * value1, int8_t * value2, int8_t * value3, int8_t * value4) {
  if (!Serial.available()) {
    return 0;
  }
  
  uint8_t buffer[ PACKET_SIZE ] = {0, 0, 0, 0, 0, 0};
  Serial.readBytes(buffer, PACKET_SIZE);
  *name                          = buffer[0];
  *value1                        = buffer[1];
  *value2                        = buffer[2];
  *value3                        = buffer[3];
  *value4                        = buffer[4];
  
  return 1;
}

void sendPacket (int8_t * name, int8_t * value1, int8_t * value2, int8_t * value3, int8_t * value4) {
  Serial.write(*name);
  Serial.write(*value1);
  Serial.write(*value2);
  Serial.write(*value3);
  Serial.write(*value4);
  Serial.write(PACKET_DELIMITER);
}

void sendLongTypePacket (int8_t * name, int32_t * value) {
  int8_t value1 = (*value >> 24) & 0xff;
  int8_t value2 = (*value >> 16) & 0xff;
  int8_t value3 = (*value >> 8)  & 0xff;
  int8_t value4 =  *value        & 0xff;
  sendPacket(name, &value1, &value2, &value3, &value4);
}

void pingPacket (int8_t * name) {
  int8_t packetToSendName = OUT_LOG_WRITE;
  int8_t value2, value3, value4;
  value2 = value3 = value4 = 0;
  sendPacket(&packetToSendName, name, &value2, &value3, &value4);
}

void logWriteFreeMemory () {
  int8_t packetToSendName = OUT_LOG_WRITE_FREE_MEMORY;
  int32_t availableMemory = freeMemory();
  sendLongTypePacket(&packetToSendName, &availableMemory);
}

void handlePacket (int8_t * name, int8_t * value1, int8_t * value2, int8_t * value3, int8_t * value4) {
  switch (*name) {
    case IN_SET_DIRECTION_AND_ACCELERATION : {
      setDirection(value1);
      setAcceleration(value2);
      updateMotorsPulse();
    }; break;
    case IN_SET_FORWARD                    : {
      gear = 1;
      updateMotorsPulse();
    }; break;
    case IN_SET_NEUTRAL                    : {
      gear = 0;
      updateMotorsPulse();
    }; break;
    case IN_SET_REVERSE                    : {
      gear = -1;
      updateMotorsPulse();
    }; break;
    case IN_START_BRAKING                  : {
      braking = 1;
      brake();
      updateLateralLightsState();
    }; break;
    case IN_STOP_BRAKING                   : {
      braking = 0;
      updateLateralLightsState();
    }; break;
    case IN_ACCELERATE                     : {
      acceleration        = 255;
      digitalAcceleration = 1;
      updateMotorsPulse();
    }; break;
    case IN_DECELERATE                     : {
      acceleration        = 0;
      digitalAcceleration = 0;
      updateMotorsPulse();
    }; break;
    case IN_SET_MOTORS_LEVEL_FACTOR        : {
      setMotorsLevelFactor(value1);
    }; break;
    case IN_SET_DIRECTION_FOR_ROTATION     : {
      setDirectionForRotation(value1);
    }; break;
    case IN_TOGGLE_LEFT_BLINKER            : {
      leftBlinkerOn  = !leftBlinkerOn;
      rightBlinkerOn = 0;
      updateLateralLightsState();
    }; break;
    case IN_TOGGLE_HEADLIGHTS              : {
      toggleHeadlights();
    }; break;
    case IN_START_HORN                     : {
      hornOn = 1;
      NewTone(BEEP_PIN, HORN_FREQUENCY);
    }; break;
    case IN_STOP_HORN                      : {
      hornOn = 0;
      noNewTone(BEEP_PIN);
    }; break;
    case IN_TOGGLE_SIREN                   : {
      toggleSiren();
      updateLateralLightsState();
    }; break;
    case IN_TOGGLE_EMERGENCY_LIGHTS        : {
      emergencyLightsOn = !emergencyLightsOn;
      updateLateralLightsState();
    }; break;
    case IN_TOGGLE_RIGHT_BLINKER           : {
      leftBlinkerOn  = 0;
      rightBlinkerOn = !rightBlinkerOn;
      updateLateralLightsState();
    };
  }
}

void setDirection (int8_t * newDirection) {
  if (*newDirection == -1) {
    direction = 254;
  } else {
    direction = *newDirection;
  }
}

void setAcceleration (int8_t * newAcceleration) {
  if (digitalAcceleration) {
    return;
  }

  acceleration = *newAcceleration;
}

void brake () {
  leftMotorsPulse  = leftMotorsFinalPulse  = 0;
  rightMotorsPulse = rightMotorsFinalPulse = 0;
}

void setMotorsLevelFactor (int8_t * newMotorsLevelFactor) {
  if (*newMotorsLevelFactor == 0) {
    motorsLevelFactor = 1;
  } else {
    motorsLevelFactor = *newMotorsLevelFactor;
  }
}

void setDirectionForRotation (int8_t * newDirectionForRotation) {
  if (*newDirectionForRotation < 1) {
    directionForRotation = 1;
  } else 
  if (*newDirectionForRotation > 127) {
    directionForRotation = 127;
  } else {
    directionForRotation = *newDirectionForRotation;
  }
}

void toggleHeadlights () {
  headlightsOn = !headlightsOn;

  if (headlightsOn) {
    headlightsFinalPulse = 255;
  } else {
    headlightsFinalPulse = 0;
  }
}

void toggleSiren () {
  sirenOn = !sirenOn;

  if (sirenOn) {
    sirenFrequency   = SIREN_MIN_FREQUENCY;
    sirenLevelFactor = abs(sirenLevelFactor);
  }
}

void updateLateralLightsState () {
  if (braking) {
    leftLightsOn       = rightLightsOn       = 1;
    leftLightsBlinking = rightLightsBlinking = 0;
  } else 
  if (sirenOn) {
    leftLightsOn       = 1;
    rightLightsOn      = 0;
    leftLightsBlinking = rightLightsBlinking = 1;
  } else 
  if (emergencyLightsOn) {
    leftLightsOn       = rightLightsOn       = 1;
    leftLightsBlinking = rightLightsBlinking = 1;
  } else 
  if (leftBlinkerOn) {
    leftLightsOn        = 1;
    rightLightsOn       = 0;
    leftLightsBlinking  = 1;
    rightLightsBlinking = 0;
  } else 
  if (rightBlinkerOn) {
    leftLightsOn        = 0;
    rightLightsOn       = 1;
    leftLightsBlinking  = 0;
    rightLightsBlinking = 1;
  } else {
    leftLightsOn = rightLightsOn = leftLightsBlinking = rightLightsBlinking = 0;
  }

  lightsCount = 0;
}

void updateMotorsPulse () {
  int8_t directionDifference = 127 - direction;
  
  if (braking) {
    if (directionDifference <= -directionForRotation) {
      leftMotorsFinalPulse  =  255 * gear;
      rightMotorsFinalPulse = -255 * gear;
    } else 
    if (directionDifference >=  directionForRotation) {
      leftMotorsFinalPulse  = -255 * gear;
      rightMotorsFinalPulse =  255 * gear;
    } else {
      brake();
    }
  } else {
    uint8_t slowestMotorAcceleration = acceleration - ((abs(directionDifference) * acceleration) / 127);
  
    if (directionDifference >= 0) {
      leftMotorsFinalPulse  = slowestMotorAcceleration * gear;
      rightMotorsFinalPulse = acceleration * gear;
    } else {
      leftMotorsFinalPulse  = acceleration * gear;
      rightMotorsFinalPulse = slowestMotorAcceleration * gear;
    }
  
    if ((leftMotorsFinalPulse >= 0  && leftMotorsPulse  >= 0) || (leftMotorsFinalPulse <= 0  && leftMotorsPulse  <= 0)) {
      leftMotorsPulse  = leftMotorsFinalPulse;
    }
  
    if ((rightMotorsFinalPulse >= 0 && rightMotorsPulse >= 0) || (rightMotorsFinalPulse <= 0 && rightMotorsPulse <= 0)) {
      rightMotorsPulse = rightMotorsFinalPulse;
    }
  }
}

void levelMotorsPulse () {
  int16_t nextLevel = 0;

  if (leftMotorsPulse < leftMotorsFinalPulse) {
    nextLevel       = leftMotorsPulse + motorsLevelFactor;
    leftMotorsPulse = (nextLevel > leftMotorsFinalPulse) ? leftMotorsFinalPulse : nextLevel ;
  } else 
  if (leftMotorsPulse > leftMotorsFinalPulse) {
    nextLevel       = leftMotorsPulse - motorsLevelFactor;
    leftMotorsPulse = (nextLevel < leftMotorsFinalPulse) ? leftMotorsFinalPulse : nextLevel ;
  }
  
  if (rightMotorsPulse < rightMotorsFinalPulse) {
    nextLevel        = rightMotorsPulse + motorsLevelFactor;
    rightMotorsPulse = (nextLevel > rightMotorsFinalPulse) ? rightMotorsFinalPulse : nextLevel ;
  } else 
  if (rightMotorsPulse > rightMotorsFinalPulse) {
    nextLevel        = rightMotorsPulse - motorsLevelFactor;
    rightMotorsPulse = (nextLevel < rightMotorsFinalPulse) ? rightMotorsFinalPulse : nextLevel ;
  }

  digitalWrite(LEFT_MOTORS_DIRECTION_PIN,  (leftMotorsPulse  >= 0) ? HIGH : LOW );
  digitalWrite(RIGHT_MOTORS_DIRECTION_PIN, (rightMotorsPulse >= 0) ? HIGH : LOW );
  analogWrite(LEFT_MOTORS_PIN,  abs(leftMotorsPulse));
  analogWrite(RIGHT_MOTORS_PIN, abs(rightMotorsPulse));
}

void levelHeadlightsPulse () {
  int16_t nextLevel = 0;

  if (headlightsPulse < headlightsFinalPulse) {
    nextLevel = headlightsPulse + HEADLIGHTS_LEVEL_FACTOR;
    headlightsPulse = (nextLevel > headlightsFinalPulse) ? headlightsFinalPulse : nextLevel ;
    analogWrite(HEADLIGHTS_PIN, headlightsPulse);
  } else 
  if (headlightsPulse > headlightsFinalPulse) {
    nextLevel = headlightsPulse - HEADLIGHTS_LEVEL_FACTOR;
    headlightsPulse = (nextLevel < headlightsFinalPulse) ? headlightsFinalPulse : nextLevel ;
    analogWrite(HEADLIGHTS_PIN, headlightsPulse);
  }
}

void updateSirenFrequency () {
  if (hornOn || !sirenOn) {
    return;
  }

  int16_t nextLevel = sirenFrequency + sirenLevelFactor;

  if (nextLevel > SIREN_MAX_FREQUENCY) {
    sirenFrequency   =  SIREN_MAX_FREQUENCY;
    sirenLevelFactor = -sirenLevelFactor;
  } else 
  if (nextLevel < SIREN_MIN_FREQUENCY) {
    sirenFrequency   =  SIREN_MIN_FREQUENCY;
    sirenLevelFactor = -sirenLevelFactor;
  } else {
    sirenFrequency = nextLevel;
  }

  NewTone(BEEP_PIN, sirenFrequency);
}

void updateBackUpBeep () {
  if (hornOn || sirenOn) {
    return;
  }
  
  if (gear > -1 || acceleration <= 0) {
    backUpBeep      = 0;
    backUpBeepCount = BACK_UP_BEEP_MAX_COUNT;
    noNewTone(BEEP_PIN);
  } else {
    backUpBeepCount++;
  
    if (backUpBeepCount < BACK_UP_BEEP_MAX_COUNT) {
      return;
    }
  
    backUpBeepCount = 0;
    backUpBeep      = !backUpBeep;
  
    if (backUpBeep) {
      NewTone(BEEP_PIN, BACK_UP_BEEP_FREQUENCY);
    } else {
      noNewTone(BEEP_PIN);
    }
  }
}

void updateLateralLights () {
  lightsCount++;

  if (lightsCount >= LIGHTS_MAX_COUNT) {
    lightsCount = 0;
    
    if (leftLightsBlinking) {
      leftLightsOn = !leftLightsOn;
    }
  
    if (rightLightsBlinking) {
      rightLightsOn = !rightLightsOn;
    }
  }

  digitalWrite(LEFT_LIGHTS_PIN,  (leftLightsOn)  ? HIGH : LOW );
  digitalWrite(RIGHT_LIGHTS_PIN, (rightLightsOn) ? HIGH : LOW );
}

void updateCollisionSectorsDistances () {
  northWestCollisionSectorDistance = northWestUSS.ping_cm();
  northEastCollisionSectorDistance = northEastUSS.ping_cm();
  southWestCollisionSectorDistance = southWestUSS.ping_cm();
  southEastCollisionSectorDistance = southEastUSS.ping_cm();
}

void reportCollisionSectorsDistances () {
  collisionSectorsDistancesReportCount++;

  if (collisionSectorsDistancesReportCount < COLLISION_SECTORS_DISTANCES_REPORT_MAX_COUNT) {
    return;
  }

  collisionSectorsDistancesReportCount = 0;
  int8_t packetToSendName              = OUT_UPDATE_COLLISION_SECTORS;
  int8_t northWestCollisionSector      = min((northWestCollisionSectorDistance * 127) / collisionSectorsMaxDistance, 127);
  int8_t northEastCollisionSector      = min((northEastCollisionSectorDistance * 127) / collisionSectorsMaxDistance, 127);
  int8_t southWestCollisionSector      = min((southWestCollisionSectorDistance * 127) / collisionSectorsMaxDistance, 127);
  int8_t southEastCollisionSector      = min((southEastCollisionSectorDistance * 127) / collisionSectorsMaxDistance, 127);
  sendPacket(&packetToSendName, &northWestCollisionSector, &northEastCollisionSector, &southWestCollisionSector, &southEastCollisionSector);
}

void setup () {
  Serial.begin(SERIAL_PORT_BPS);

  pinMode(LEFT_MOTORS_DIRECTION_PIN,    OUTPUT);
  pinMode(RIGHT_MOTORS_DIRECTION_PIN,   OUTPUT);
  pinMode(LEFT_MOTORS_PIN,              OUTPUT);
  pinMode(RIGHT_MOTORS_PIN,             OUTPUT);
  pinMode(BEEP_PIN,                     OUTPUT);
  pinMode(HEADLIGHTS_PIN,               OUTPUT);
  pinMode(LEFT_LIGHTS_PIN,              OUTPUT);
  pinMode(RIGHT_LIGHTS_PIN,             OUTPUT);
  pinMode(NORTH_WEST_USS_TRIG_PIN,      OUTPUT);
  pinMode(NORTH_WEST_USS_ECHO_PIN,      INPUT);
  pinMode(NORTH_EAST_USS_TRIG_PIN,      OUTPUT);
  pinMode(NORTH_EAST_USS_ECHO_PIN,      INPUT);
  pinMode(SOUTH_WEST_USS_TRIG_PIN,      OUTPUT);
  pinMode(SOUTH_WEST_USS_ECHO_PIN,      INPUT);
  pinMode(SOUTH_EAST_USS_TRIG_PIN,      OUTPUT);
  pinMode(SOUTH_EAST_USS_ECHO_PIN,      INPUT);
  
  digitalWrite(LEFT_MOTORS_DIRECTION_PIN,  HIGH);
  digitalWrite(RIGHT_MOTORS_DIRECTION_PIN, HIGH);
  digitalWrite(LEFT_MOTORS_PIN,             LOW);
  digitalWrite(RIGHT_MOTORS_PIN,            LOW);
  digitalWrite(HEADLIGHTS_PIN,              LOW);
  digitalWrite(LEFT_LIGHTS_PIN,             LOW);
  digitalWrite(RIGHT_LIGHTS_PIN,            LOW);
  digitalWrite(NORTH_WEST_USS_TRIG_PIN,     LOW);
  digitalWrite(NORTH_WEST_USS_ECHO_PIN,     LOW);
  digitalWrite(NORTH_EAST_USS_TRIG_PIN,     LOW);
  digitalWrite(NORTH_EAST_USS_ECHO_PIN,     LOW);
  digitalWrite(SOUTH_WEST_USS_TRIG_PIN,     LOW);
  digitalWrite(SOUTH_WEST_USS_ECHO_PIN,     LOW);
  digitalWrite(SOUTH_EAST_USS_TRIG_PIN,     LOW);
  digitalWrite(SOUTH_EAST_USS_ECHO_PIN,     LOW);
  
  noNewTone(BEEP_PIN);
}

void loop () {
  int8_t packetName, packetValue1, packetValue2, packetValue3, packetValue4;
  
  if (readPacket(&packetName, &packetValue1, &packetValue2, &packetValue3, &packetValue4)) {
    pingPacket(&packetName);
    handlePacket(&packetName, &packetValue1, &packetValue2, &packetValue3, &packetValue4);
  }

  levelMotorsPulse();
  levelHeadlightsPulse();
  updateSirenFrequency();
  updateBackUpBeep();
  updateLateralLights();
  updateCollisionSectorsDistances();
  reportCollisionSectorsDistances();
  
  delay(CLOCK_TIME);
}

