/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// Update OSD function
//  - handles timing and calling display functions of only enabled features
///////////////////////////////////////////////////////////////////////////////

uint32_t timerOSD = 0x00000000;

void updateMax7456(uint32_t currentOSDTime, uint8_t updateOSD)
{
	if (eepromConfig.osdEnabled)
	{
		if ((timerOSD & 0x80008000) || updateOSD)  // 3.125Hz
		{
			if (eepromConfig.osdDisplayAlt || eepromConfig.osdDisplayAltHoldState)
				displayAltitude(sensors.pressureAlt50Hz, 0.0f, verticalModeState, updateOSD);
		}

		if ((timerOSD & 0x55555555) || updateOSD)  // 25Hz
        {
			if (eepromConfig.osdDisplayAH)
				displayArtificialHorizon(sensors.attitude500Hz[ROLL], sensors.attitude500Hz[PITCH], flightMode, updateOSD);

			else if (eepromConfig.osdDisplayAtt)
				displayAttitude(sensors.attitude500Hz[ROLL], sensors.attitude500Hz[PITCH], flightMode, updateOSD);
		}


		if ((timerOSD & 0x20202020) || updateOSD)  // 6.25Hz
		{
			if (eepromConfig.osdDisplayHdg || eepromConfig.osdDisplayHdgBar)
				displayHeading(heading.mag, updateOSD);
		}

		if ((timerOSD & 0x00800080) || updateOSD)  // 3.125Hz
		{
			if (eepromConfig.osdDisplayTimer)
				displayMotorArmedTime(currentOSDTime, updateOSD);
		}

		if ((timerOSD & 0x08000800) || updateOSD)  // 3.125Hz
		{
			if (eepromConfig.osdDisplayVoltage || eepromConfig.osdDisplayCurrent)
				displayBattery(updateOSD);
		}

		if ((timerOSD & 0x4A4A4A4A) || updateOSD)  // 18.75Hz (3/8 cycles, 3/8 * 50)
		{
			if (eepromConfig.osdDisplayThrot)
				displayThrottle(updateOSD);
		}

		if ((timerOSD & 0x00800080) || updateOSD)  // 3.125Hz
		{
			if (eepromConfig.osdDisplayRSSI)
				displayRSSI(updateOSD);
		}

		timerOSD <<= 1;
		if (!timerOSD)
			timerOSD = 0x01;
	}
}

///////////////////////////////////////////////////////////////////////////////
// AltitudeHold Display
///////////////////////////////////////////////////////////////////////////////

uint8_t lastHoldState    = 99;

int16_t lastAltitude            = 12345;     // bogus value to force update
int16_t lastAltitudeRef         = 12345;     // bogus value to force update

int16_t lastVerticalVelocity    = 12345;     // bogus value to force update
int16_t lastVerticalVelocityRef = 12345;     // bogus value to force update

void displayAltitude(float pressureAltitude, float altitudeReference, uint8_t verticalModeState, uint8_t update)
{
    int16_t currentAltitude;
    int16_t currentAltitudeRef;

    if (eepromConfig.metricUnits)
    {
		currentAltitude    = (int16_t)(pressureAltitude  * 10.0f);
		currentAltitudeRef = (int16_t)(altitudeReference * 10.0f);
	}
	else
	{
		currentAltitude    = (int16_t)(pressureAltitude  * 3.281f);
        currentAltitudeRef = (int16_t)(altitudeReference * 3.281f);
	}

    if ( (lastAltitude != currentAltitude)  || update)
    {
        char    buf[7];
        if (eepromConfig.metricUnits)
        {
		    if (abs(currentAltitude) < 100)
		    {
                snprintf(buf,7,"\011%c%1d.%1dm",currentAltitude < 0 ? '-' : ' ', abs(currentAltitude/10),abs(currentAltitude%10));
            }
            else
            {
                snprintf(buf,7,"\011%4dm",currentAltitude / 10);
            }
        }
        else
        {
		    snprintf(buf,7,"\011%4df",currentAltitude);
	    }

        writeMax7456Chars(buf, 6, 0, eepromConfig.osdDisplayAltRow, eepromConfig.osdDisplayAltCol);

        lastAltitude = currentAltitude;
    }

    // Vertical mode handling:
    // - show altitude ref when active

    if (eepromConfig.osdDisplayAltHoldState)
    {
        bool    isWriteNeeded = false;
        char    buf[7];

		if (verticalModeState == ALT_DISENGAGED_THROTTLE_ACTIVE)
		{
		    if (lastHoldState != ALT_DISENGAGED_THROTTLE_ACTIVE)
			{
				lastHoldState = ALT_DISENGAGED_THROTTLE_ACTIVE;
				memset(buf,0,6);
				isWriteNeeded = true;
			}
		}

		if ((verticalModeState == ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT)             ||
			(verticalModeState == ALT_HOLD_AT_REFERENCE_ALTITUDE)               ||
			(verticalModeState == VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY) ||
			(verticalModeState == ALT_DISENGAGED_THROTTLE_INACTIVE))
		{
			if ((lastHoldState != verticalModeState) ||
				(lastAltitudeRef != currentAltitudeRef))
		    {
			    lastHoldState   = verticalModeState;
			    lastAltitudeRef = currentAltitudeRef;

			    if (eepromConfig.metricUnits)
			    {
			        if (abs(currentAltitudeRef) < 100)
			        {
			            snprintf(buf,7,"\012%c%1d.%1dm", currentAltitudeRef < 0 ? '-' : ' ',abs(currentAltitudeRef / 10), abs(currentAltitudeRef % 10));
			        }
			        else
			        {
			            snprintf(buf,7,"\012%4dm",currentAltitudeRef / 10);
			        }
			    }
			    else
			    {
			        snprintf(buf,7,"\12%4df",currentAltitudeRef);
			    }

			    isWriteNeeded = true;
		    }
	    }

		if (isWriteNeeded)
	        writeMax7456Chars(buf, 6, 0, eepromConfig.osdDisplayAltRow, eepromConfig.osdDisplayAltCol+6);
    }

}

///////////////////////////////////////////////////////////////////////////////
// Artificial Horizon Display
///////////////////////////////////////////////////////////////////////////////

// 012345678901234567890123456789
//
//         - - - RR - - -

#define LINE_ROW_0 0x80            // character address of a character with a horizontal line in row 0. Other rows follow this one
#define AH_MAX_PITCH_ANGLE (PI/8)  // bounds of scale used for displaying pitch.
                                   // when pitch is >= |this number|, the pitch lines will be at top or bottom of bounding box
#define RETICLE_COL 14             // reticle will be in this col, and col to the right

// columns where the roll line is printed
static const uint8_t ahColumns[6] = {8,10,12,17,19,21};

uint8_t  reticleRow;
uint8_t  ahTopPixel;
uint8_t  ahBottomPixel;
uint8_t  ahCenter;

uint8_t ahOldLine[6]   = {0,0,0,0,0,0};
uint8_t lastAHflightMode = 25;

void displayArtificialHorizon(float roll, float pitch, uint8_t flightMode, uint8_t updateOSD)
{
	char    reticle[2];
	char    rollLine;
    uint8_t i;
    uint8_t row;

    for (i = 0; i < 6; i++)
    {
        row = constrain(ahCenter +
			           (14.5 - (float)ahColumns[i]) * 12 * 1.4 * roll +
			           (pitch/AH_MAX_PITCH_ANGLE*(ahCenter - ahTopPixel)),
			            ahTopPixel, ahBottomPixel);

        if ((row/18) != ahOldLine[i])
        {
            writeMax7456Chars(NULL, 1, 0, ahOldLine[i], ahColumns[i]);
            ahOldLine[i] = row/18;
        }

        rollLine = LINE_ROW_0 + (row % 18);
        writeMax7456Chars(&rollLine, 1, 0, ahOldLine[i], ahColumns[i]);
    }

    // Reticle on the center of the screen
    // 0 - rate mode (no letter)
    // 1 - Attitude 'S'
    // 2 - GPS position hold 'P'
    // 3 - GPS navigation 'N'

    if ((lastAHflightMode != flightMode) || updateOSD)
    {
        reticle[0] = flightMode * 2 + 1;
        reticle[1] = reticle[0] + 1;

        //write 2 chars to row (middle), column 14
        writeMax7456Chars(reticle, 2, 0, reticleRow, RETICLE_COL);

        lastAHflightMode = flightMode;
    }
}

//////////////////////////////////////////////////////////////////////////////
// Attitude Display
//////////////////////////////////////////////////////////////////////////////

#define AI_MAX_PITCH_ANGLE (PI/4)  // Bounds of scale used for displaying pitch.
                                   // when pitch is >= |this number|, the pitch lines will be at top or bottom of bounding box
#define PITCH_L_COL 7
#define PITCH_R_COL 22

// columns where the roll line is printed
static const uint8_t ROLL_COLUMNS[4] = {10,12,17,19};

uint8_t aiTopPixel;
uint8_t aiBottomPixel;
uint8_t aiCenter;

uint8_t aiOldline[5] = {0,0,0,0,0};
uint8_t lastATTflightMode = 9;

void displayAttitude(float roll, float pitch, uint8_t flightMode, uint8_t updateOSD)
{
	char     pitchLine;
	char     reticle[2];
	char     rollLine;
	float    gradient;
	uint8_t  aiRows[5] = {0,0,0,0,0};  //Holds the row, in pixels, of AI elements: pitch then roll from left to right.
    uint8_t  i;
    uint16_t distFar;
    uint16_t distNear;

    //Calculate row of new pitch lines
    aiRows[0] = constrain((int)aiCenter +
    		              (int)((pitch / AI_MAX_PITCH_ANGLE) * (aiCenter - aiTopPixel)),
    		               aiTopPixel, aiBottomPixel);

    pitchLine = LINE_ROW_0 + (aiRows[0] % 18);

    if (aiOldline[0] != aiRows[0] / 18)
    {
        //Remove old pitch lines if not overwritten by new ones
        writeMax7456Chars(NULL, 1, 0, aiOldline[0], PITCH_L_COL);
        writeMax7456Chars(NULL, 1, 0, aiOldline[0], PITCH_R_COL);
        aiOldline[0] = aiRows[0] / 18;
    }

    //Write new pitch lines
    writeMax7456Chars(&pitchLine, 1, 0, aiOldline[0], PITCH_L_COL);
    writeMax7456Chars(&pitchLine, 1, 0, aiOldline[0], PITCH_R_COL);

    //Calculate row (in pixels) of new roll lines
    distFar  = (ROLL_COLUMNS[3] - (RETICLE_COL + 1))*12 + 6; //horizontal pixels between centre of reticle and centre of far angle line
    distNear = (ROLL_COLUMNS[2] - (RETICLE_COL + 1))*12 + 6;
    gradient = 1.4f * roll; // was "tan(roll)", yes rude but damn fast !!

    aiRows[3] = constrain(aiCenter - (int)(((float)distNear) * gradient), aiTopPixel, aiBottomPixel);
    aiRows[4] = constrain(aiCenter - (int)(((float)distFar)  * gradient), aiTopPixel, aiBottomPixel);
    aiRows[1] = constrain( 2 * aiCenter - aiRows[4], aiTopPixel, aiBottomPixel);
    aiRows[2] = constrain( 2 * aiCenter - aiRows[3], aiTopPixel, aiBottomPixel);

    //writing new roll lines to screen
    for (i = 1; i < 5; i++ )
    {
        // clear previous roll lines if not going to overwrite
        if (aiOldline[i] != aiRows[i] / 18)
        {
            writeMax7456Chars(NULL, 1, 0, aiOldline[i], ROLL_COLUMNS[i-1]);
            aiOldline[i] = aiRows[i]/18;
        }

        //converting rows (in pixels) to character addresses used for the 'lines'
        rollLine = LINE_ROW_0 + (aiRows[i] % 18);
        writeMax7456Chars(&rollLine, 1, 0, aiOldline[i], ROLL_COLUMNS[i-1]);
    }

    // Reticle on the center of the screen
    // 0 - rate mode (no letter)
    // 1 - Attitude 'S'
    // 2 - GPS position hold 'P'
    // 3 - GPS navigation 'N'

    if ((lastATTflightMode != flightMode) ||  updateOSD)
    {
        reticle[0] = flightMode * 2 + 1;
        reticle[1] = reticle[0] + 1;
        writeMax7456Chars(reticle, 2, 0, reticleRow, RETICLE_COL); //write 2 chars to row (middle), column 14
        lastATTflightMode = flightMode;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Heading Display
///////////////////////////////////////////////////////////////////////////////

int16_t lastOSDheading = 361; // bogus value to force update
// N = 0x4e; E = 0x45; S = 0x53; W = 0x57; - = 0x2d; | = 0x7c;
static char headingBarShown[12];
const char headingBar[36] = {0x4e,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d,
                           0x45,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d,
                           0x53,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d,
                           0x57,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d};

void displayHeading(float currentHeading, uint8_t update)
{
    int16_t currentHeadingDeg;

    currentHeadingDeg = (int16_t)(currentHeading * R2D) % 360;
    if (currentHeadingDeg < 0)
		currentHeadingDeg += 360;

    if ((currentHeadingDeg != lastOSDheading) || update)
    {

    	if (eepromConfig.osdDisplayHdg)
    	{
    		char buf[6];
			snprintf(buf ,6, "\026%3d\027", currentHeadingDeg); // \026 is compass \027 is degree symbol
			writeMax7456Chars(buf, 5, 0, eepromConfig.osdDisplayHdgRow, eepromConfig.osdDisplayHdgCol);
    	}

        if (eepromConfig.osdDisplayHdgBar) {
			int8_t lastPos;
			int8_t currentPos;

			currentPos = currentHeadingDeg / 10;
			lastPos = lastOSDheading / 10;

			if ((currentPos != lastPos) || update)
			{
			    uint8_t x = 0;
				currentPos -= 5;

				if (currentPos < 0)
					currentPos += 36;

				for (x = 0; x <= 10; ++x)
				{
					headingBarShown[x] = headingBar[currentPos];

					if (++currentPos > 35)
						currentPos = 0;
				}

				headingBarShown[11] = '\0';
				writeMax7456Chars(headingBarShown, 11, 0, eepromConfig.osdDisplayHdgBarRow, eepromConfig.osdDisplayHdgBarCol);
			}
		}

        lastOSDheading = currentHeadingDeg;
    }
}


/////////////////////////////////////////////////////// ////////////////////////
// Battery Display
///////////////////////////////////////////////////////////////////////////////
float osdVoltageLast = 10000000.0f;
float osdCurrentLast = 10000000.0f;

void displayBattery(uint8_t updateOSD)
{
    char buf[20];
    uint8_t  osdVoltage     = batteryVoltage * 10.0f;
    uint8_t  osdCurrent     = batteryCurrent * 10.0f;
    uint16_t osdCurrentUsed = batteryCurrentUsed;


    if (eepromConfig.osdDisplayVoltage && ((osdVoltage != osdVoltageLast) || updateOSD))
    {
	    snprintf(buf, sizeof(buf), "\20%2d.%1dV", (uint8_t)(osdVoltage / 10), (uint8_t)(osdVoltage % 10));
	    writeMax7456Chars(buf, 7, ((batteryVoltage / (float)eepromConfig.batteryCells) < eepromConfig.batteryLow)?1:0,
	    		eepromConfig.osdDisplayVoltageRow, eepromConfig.osdDisplayVoltageCol);
	    osdVoltageLast = osdVoltage;
    }

    if (eepromConfig.osdDisplayCurrent && eepromConfig.currentMonitoring)
    {
    	if ((osdCurrent != osdCurrentLast) || updateOSD)
    	{
    		if ((osdCurrent / 10) >= 10)  // when >10A, display whole amps, otherwise display tenths
    		{
    			snprintf(buf, sizeof(buf), "%4dA%5f\24  ", (uint16_t)osdCurrent, (float)batteryCurrentUsed);
    		}
			else
			{
				snprintf(buf, sizeof(buf), "%1d.%1dA%5d\24  ", (uint8_t)(osdCurrent / 10), (uint8_t)(osdCurrent % 10), osdCurrentUsed);
			}
    		osdCurrentLast = osdCurrent;
    		writeMax7456Chars(buf, 11, 0, eepromConfig.osdDisplayCurrentRow, eepromConfig.osdDisplayCurrentCol);
    	}

    }
}



///////////////////////////////////////////////////////////////////////////////
// RSSI Display
///////////////////////////////////////////////////////////////////////////////
uint8_t lastRSSI = 0;
void displayRSSI(uint8_t updateOSD)
{
	if ((RSSI != lastRSSI) || updateOSD)
	{
		char buf[20];
		snprintf(buf,sizeof(buf),"\372%3u%%",RSSI);
		writeMax7456Chars(buf,5, (eepromConfig.rssiWarning > RSSI)?1:0, eepromConfig.osdDisplayRSSIRow, eepromConfig.osdDisplayRSSICol);

		lastRSSI = RSSI;
	}
}

///////////////////////////////////////////////////////////////////////////////
// Throttle Display
///////////////////////////////////////////////////////////////////////////////
uint16_t lastThrottle = 0; // force update on first iteration

void displayThrottle(uint8_t update)
{
    if ((rxCommand[THROTTLE] != lastThrottle) || update)
    {
        char buf[5];
		snprintf(buf,5,"%d", (uint16_t)(rxCommand[THROTTLE] / 2));
		writeMax7456Chars(buf,4,0,eepromConfig.osdDisplayThrotRow,eepromConfig.osdDisplayThrotCol);
		lastThrottle = rxCommand[THROTTLE];
    }

}

///////////////////////////////////////////////////////////////////////////////
// Motors Armed Timer Display
///////////////////////////////////////////////////////////////////////////////
uint32_t previousTime = 0;
uint16_t previousArmedTimeSeconds = 500;
uint32_t armedTime = 0;

void displayMotorArmedTime(uint32_t currentOSDTime, uint8_t updateOSD)
{

	if (armed || updateOSD)
	{
		uint32_t deltaOSDTime = currentOSDTime - previousTime;

		if ((previousTime == 0) || (deltaOSDTime > 350000)) // updated 3.125Hz = every 320mS = 320,000uS
		{
			previousTime = currentOSDTime;
			deltaOSDTime = 0;
		}

		armedTime += (deltaOSDTime);
	}
	else
		return;

	if (currentOSDTime != 0)
		previousTime = currentOSDTime;

	uint16_t armedTimeSeconds = armedTime / 1000000;

	if ((armedTimeSeconds != previousArmedTimeSeconds) || updateOSD)
	{
		previousArmedTimeSeconds = armedTimeSeconds;
		char buf[7];
		snprintf(buf, 7, "\025%02u:%02u", armedTimeSeconds / 60, armedTimeSeconds % 60);
		writeMax7456Chars(buf, 6, 0, eepromConfig.osdDisplayTimerRow, eepromConfig.osdDisplayTimerCol);
	}

}

///////////////////////////////////////////////////////////////////////////////

