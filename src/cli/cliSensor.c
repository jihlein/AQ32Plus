/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the AQ32 Flight Control Board

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX

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
// Sensor CLI
///////////////////////////////////////////////////////////////////////////////

void sensorCLI()
{
    uint8_t  sensorQuery = 'x';
    uint8_t  tempInt;
    uint8_t  validQuery = false;

    cliBusy = true;

    cliPrint("\nEntering Sensor CLI....\n\n");

    while(true)
    {
        cliPrint("Sensor CLI -> ");

		while ((cliAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    sensorQuery = cliRead();

		cliPrint("\n");

		switch(sensorQuery)
		{
            ///////////////////////////

            case 'a': // Sensor Data
            	cliPrintF("\n");
            	cliPrintF("External HMC5883 in use:   %s\n", eepromConfig.externalHMC5883 ? "Yes" : "No");
            	cliPrintF("External MS5611  in use:   %s\n", eepromConfig.externalMS5611  ? "Yes" : "No");
            	cliPrintF("\n");

            	cliPrintF("Accel Temp Comp Slope:     %9.4f, %9.4f, %9.4f\n",   eepromConfig.accelTCBiasSlope[XAXIS],
                                                		                        eepromConfig.accelTCBiasSlope[YAXIS],
                                                		                        eepromConfig.accelTCBiasSlope[ZAXIS]);
                cliPrintF("Accel Temp Comp Bias:      %9.4f, %9.4f, %9.4f\n",   eepromConfig.accelTCBiasIntercept[XAXIS],
                                                		                        eepromConfig.accelTCBiasIntercept[YAXIS],
                                                		                        eepromConfig.accelTCBiasIntercept[ZAXIS]);
                cliPrintF("Gyro Temp Comp Slope:      %9.4f, %9.4f, %9.4f\n",   eepromConfig.gyroTCBiasSlope[ROLL ],
                                                                		        eepromConfig.gyroTCBiasSlope[PITCH],
                                                                		        eepromConfig.gyroTCBiasSlope[YAW  ]);
                cliPrintF("Gyro Temp Comp Intercept:  %9.4f, %9.4f, %9.4f\n",   eepromConfig.gyroTCBiasIntercept[ROLL ],
                                                                   		        eepromConfig.gyroTCBiasIntercept[PITCH],
                                                                   		        eepromConfig.gyroTCBiasIntercept[YAW  ]);
                cliPrintF("Mag Bias:                  %9.4f, %9.4f, %9.4f\n",   eepromConfig.magBias[XAXIS],
                                                   		                        eepromConfig.magBias[YAXIS],
                                                   		                        eepromConfig.magBias[ZAXIS]);
                cliPrintF("Accel One G:               %9.4f\n",   accelOneG);
                cliPrintF("Accel Cutoff:              %9.4f\n",   eepromConfig.accelCutoff);
                cliPrintF("KpAcc (MARG):              %9.4f\n",   eepromConfig.KpAcc);
                cliPrintF("KiAcc (MARG):              %9.4f\n",   eepromConfig.KiAcc);
                cliPrintF("KpMag (MARG):              %9.4f\n",   eepromConfig.KpMag);
                cliPrintF("KiMag (MARG):              %9.4f\n",   eepromConfig.KiMag);
                cliPrintF("hdot est/h est Comp Fil A: %9.4f\n",   eepromConfig.compFilterA);
                cliPrintF("hdot est/h est Comp Fil B: %9.4f\n",   eepromConfig.compFilterB);

                cliPrint("MPU6000 DLPF:                 ");
                switch(eepromConfig.dlpfSetting)
                {
                    case DLPF_256HZ:
                        cliPrint("256 Hz\n");
                        break;
                    case DLPF_188HZ:
                        cliPrint("188 Hz\n");
                        break;
                    case DLPF_98HZ:
                        cliPrint("98 Hz\n");
                        break;
                    case DLPF_42HZ:
                        cliPrint("42 Hz\n");
                        break;
                }

                cliPrint("Magnetic Variation:           ");
                if (eepromConfig.magVar >= 0.0f)
                  cliPrintF("E%6.4f\n\n",  eepromConfig.magVar * R2D);
                else
                  cliPrintF("W%6.4f\n\n", -eepromConfig.magVar * R2D);

                if (eepromConfig.verticalVelocityHoldOnly)
                	cliPrint("Vertical Velocity Hold Only\n\n");
                else
                	cliPrint("Vertical Velocity and Altitude Hold\n\n");

                validQuery = false;
                break;

            ///////////////////////////

            case 'b': // MPU6000 Calibration
                mpu6000Calibration();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'c': // Magnetometer Calibration
                magCalibration();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'e': // Toggle External HMC5883 Use
                if (eepromConfig.externalHMC5883)
                	eepromConfig.externalHMC5883 = false;
                else
               	    eepromConfig.externalHMC5883 = true;

                initMag();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'f': // Toggle External MS5611 Use
                if (eepromConfig.externalMS5611)
                	eepromConfig.externalMS5611 = false;
                else
               	    eepromConfig.externalMS5611 = true;

                initPressure();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'v': // Toggle Vertical Velocity Hold Only
                if (eepromConfig.verticalVelocityHoldOnly)
                	eepromConfig.verticalVelocityHoldOnly = false;
                else
               	    eepromConfig.verticalVelocityHoldOnly = true;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

        	case 'x':
			    cliPrint("\nExiting Sensor CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Set MPU6000 Digital Low Pass Filter
                tempInt = (uint8_t)readFloatCLI();

                switch(tempInt)
                {
                    case DLPF_256HZ:
                        eepromConfig.dlpfSetting = BITS_DLPF_CFG_256HZ;
                        break;

                    case DLPF_188HZ:
                    	eepromConfig.dlpfSetting = BITS_DLPF_CFG_188HZ;
                    	break;

                    case DLPF_98HZ:
                    	eepromConfig.dlpfSetting = BITS_DLPF_CFG_98HZ;
                    	break;

                    case DLPF_42HZ:
                    	eepromConfig.dlpfSetting = BITS_DLPF_CFG_42HZ;
                     	break;
                }

                setSPIdivisor(MPU6000_SPI, 64);  // 0.65625 MHz SPI clock (within 20 +/- 10%)

                GPIO_ResetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN);
			    spiTransfer(MPU6000_SPI, MPU6000_CONFIG);
			    spiTransfer(MPU6000_SPI, eepromConfig.dlpfSetting);
			    GPIO_SetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN);

                setSPIdivisor(MPU6000_SPI, 2);  // 21 MHz SPI clock (within 20 +/- 10%)

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'B': // Accel Cutoff
                eepromConfig.accelCutoff = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // kpAcc, kiAcc
                eepromConfig.KpAcc = readFloatCLI();
                eepromConfig.KiAcc = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // kpMag, kiMag
                eepromConfig.KpMag = readFloatCLI();
                eepromConfig.KiMag = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // h dot est/h est Comp Filter A/B
                eepromConfig.compFilterA = readFloatCLI();
                eepromConfig.compFilterB = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

    	    ///////////////////////////

            case 'M': // Magnetic Variation
                eepromConfig.magVar = readFloatCLI() * D2R;

                sensorQuery = 'a';
                validQuery = true;
                break;

			///////////////////////////

			case 'W': // Write EEPROM Parameters
                cliPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();

                validQuery = false;
                break;

			///////////////////////////

			case '?':
			   	cliPrint("\n");
			   	cliPrint("'a' Display Sensor Data                    'A' Set MPU6000 DLPF                     A0 thru 3, see aq32Plus.h\n");
			   	cliPrint("'b' MPU6000 Temp Calibration               'B' Set Accel Cutoff                     BAccelCutoff\n");
			   	cliPrint("'c' Magnetometer Calibration               'C' Set kpAcc/kiAcc                      CkpAcc;kiAcc\n");
			   	cliPrint("                                           'D' Set kpMag/kiMag                      DkpMag;kiMag\n");
			   	cliPrint("'e' Toggle External HMC5883 State          'E' Set h dot est/h est Comp Filter A/B  EA;B\n");
			   	cliPrint("'f' Toggle External MS5611 State           'M' Set Mag Variation (+ East, - West)   MMagVar\n");
			   	cliPrint("'v' Toggle Vertical Velocity Hold Only\n");
			   	cliPrint("                                           'W' Write EEPROM Parameters\n");
			   	cliPrint("'x' Exit Sensor CLI                        '?' Command Summary\n\n");

			   	validQuery = false;
	    	    break;

	    	///////////////////////////
	    }
	}

}

