//+--------------------------------------------------------------------------
//
// File:        ESPinfo.cpp
//
// NightDriverStrip - (c) 2018 Plummer's Software LLC.  All Rights Reserved.  
//
// This file is part of the NightDriver software project.
//
//    NightDriver is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//   
//    NightDriver is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//   
//    You should have received a copy of the GNU General Public License
//    along with Nightdriver.  It is normally found in copying.txt
//    If not, see <https://www.gnu.org/licenses/>.
//
//
// Description:
//
//    Get some info regarding the ESP board in use
//
// History:     09-Oct-2021         BobT      Commented
//
//---------------------------------------------------------------------------

#include <esp_system.h>
#include <esp_wifi.h>
#include "ESPinfo.h"

uint32_t    chipId = 0;
uint32_t    ChipRev = 0;
uint32_t    ChipCores = 0;
const char* ChipModel = 0;


void ESPinfo()
{
  ////////////////////////////////////////////
  //   Get ESChip Info                      //
  ////////////////////////////////////////////
  Serial.println("Getting Chip Info...");
  
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  
  ChipModel = ESP.getChipModel();
  ChipRev   = ESP.getChipRevision();
  ChipCores = ESP.getChipCores();

  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: "); Serial.println(chipId); Serial.println();

}