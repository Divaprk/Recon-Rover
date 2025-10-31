#pragma once
#include "pico/stdlib.h"
#include <stdbool.h>

void setupUltrasonicPins(uint trigPin, uint echoPin);
int  getCm(uint trigPin, uint echoPin);      
int  getInch(uint trigPin, uint echoPin);    
bool emergencyStopIfTooClose(uint trigPin, uint echoPin, int threshold_cm);
