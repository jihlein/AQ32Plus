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
// AltitudeHold Display
///////////////////////////////////////////////////////////////////////////////

#define ALTITUDE_ROW 1
#define ALTITUDE_COL 1

uint8_t lastHoldState    = 6;
int16_t lastAltitude     = 12345;     // bogus value to force update
int16_t lastHoldAltitude = 12345;     // bogus value to force update

void displayAltitude(float pressureAltitude, float altitudeReference, uint8_t altHoldState)
{
    bool    isWriteNeeded = false;
    char    buf[7];
    int16_t currentAltitude;
    int16_t currentHoldAltitude;

    if (eepromConfig.metricUnits)
    {
		currentAltitude     = (int16_t)(pressureAltitude  * 10.0f);
		currentHoldAltitude = (int16_t)(altitudeReference * 10.0f);
	}
	else
	{
		currentAltitude     = (int16_t)(pressureAltitude  * 3.281f);
        currentHoldAltitude = (int16_t)(altitudeReference * 3.281f);
	}

    if ( lastAltitude != currentAltitude )
    {
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

        writeMax7456Chars(buf, 6, 0, ALTITUDE_ROW, ALTITUDE_COL);

        lastAltitude = currentAltitude;
    }

    // AltitudeHold handling:
    // - show hold altitude when it is active
    // - show "panic" if 'paniced' out

    switch (altHoldState)
    {
        case DISENGAGED:
            if (lastHoldState != DISENGAGED)
            {
                lastHoldState = DISENGAGED;
                memset(buf,0,6);
                isWriteNeeded = true;
            }
            break;

        case ENGAGED:
            if ((lastHoldState != ENGAGED) || (lastHoldAltitude != currentHoldAltitude))
            {
                lastHoldState = ENGAGED;
                lastHoldAltitude = currentHoldAltitude;

                if (eepromConfig.metricUnits)
			    {
				    if (abs(currentHoldAltitude)<100)
				    {
					    snprintf(buf,7,"\012%c%1d.%1dm", currentHoldAltitude < 0 ? '-' : ' ',abs(currentHoldAltitude/10),abs(currentHoldAltitude%10));
					}
					else
					{
					    snprintf(buf,7,"\012%4dm",currentHoldAltitude/10);
					}
				}
				else
				{
					snprintf(buf,7,"\12%4df",currentHoldAltitude);
				}

				isWriteNeeded = true;
            }
            break;

        case PANIC:
            if (lastHoldState != PANIC)
            {
                lastHoldState = PANIC;
                snprintf(buf,7,"\12panic");
                isWriteNeeded = true;
            }
            break;
    }

    if (isWriteNeeded)
        writeMax7456Chars(buf, 6, 0, ALTITUDE_ROW, ALTITUDE_COL+6);
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

void displayArtificialHorizon(float roll, float pitch, uint8_t flightMode)
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

    if (lastAHflightMode != flightMode)
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

void displayAttitude(float roll, float pitch, uint8_t flightMode)
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

    aiRows[1] = constrain( 2 * aiCenter - aiRows[4], aiTopPixel, aiBottomPixel);
    aiRows[2] = constrain( 2 * aiCenter - aiRows[3], aiTopPixel, aiBottomPixel);
    aiRows[3] = constrain(aiCenter - (int)(((float)distNear) * gradient), aiTopPixel, aiBottomPixel);
    aiRows[4] = constrain(aiCenter - (int)(((float)distFar)  * gradient), aiTopPixel, aiBottomPixel);

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

    if (lastATTflightMode != flightMode)
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

#define COMPASS_ROW 1
#define COMPASS_COL 13

int16_t lastOSDheading = 361; // bogus value to force update

void displayHeading(float currentHeading)
{
	char buf[6];
    int16_t currentHeadingDeg;

    currentHeadingDeg = (int16_t)((currentHeading * 180.0f / PI) + 360.0f) % 360;

    if (currentHeadingDeg != lastOSDheading)
    {
    	snprintf(buf ,6, "\026%3d\027", currentHeadingDeg); // \026 is compass \027 is degree symbol

    	writeMax7456Chars(buf, 5, 0, COMPASS_ROW, COMPASS_COL);

        lastOSDheading = currentHeadingDeg;
    }
}

///////////////////////////////////////////////////////////////////////////////


