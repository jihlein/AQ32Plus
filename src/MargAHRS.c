///=====================================================================================================
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//
//  1 June 2012 Modified by J. Ihlein
// 27 Aug  2012 Extensively modified to include G.K. Egan's accel confidence calculations and
//                                                          calculation efficiency updates
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Header files

#include "board.h"

//----------------------------------------------------------------------------------------------------
// Variable definitions

float exAcc    = 0.0f,    eyAcc = 0.0f,    ezAcc = 0.0f; // accel error
float exAccInt = 0.0f, eyAccInt = 0.0f, ezAccInt = 0.0f; // accel integral error

float exMag    = 0.0f, eyMag    = 0.0f, ezMag    = 0.0f; // mag error
float exMagInt = 0.0f, eyMagInt = 0.0f, ezMagInt = 0.0f; // mag integral error

float kpAcc, kiAcc;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// auxiliary variables to reduce number of repeated operations
float q0q0, q0q1, q0q2, q0q3;
float q1q1, q1q2, q1q3;
float q2q2, q2q3;
float q3q3;

float halfT;

uint8_t MargAHRSinitialized = false;

//----------------------------------------------------------------------------------------------------

float accConfidenceDecay = 0.0f;
float accConfidence      = 1.0f;

#define HardFilter(O,N)  (O*0.9f+N*0.1f)

void calculateAccConfidence(float accMag)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	static float accMagP = 1.0f;

	accMag /= accelOneG;  // HJI Added to convert MPS^2 to G's

	accMag  = HardFilter(accMagP, accMag );
	accMagP = accMag;

	accConfidence
			= constrain(1.0 - (accConfidenceDecay * sqrt(fabs(accMag - 1.0f))), 0.0f, 1.0f);

}

//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Initialization
//====================================================================================================

void MargAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll  = atan2(-ay, -az);
    initialPitch = atan2( ax, -az);

    cosRoll  = cosf(initialRoll);
    sinRoll  = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

//====================================================================================================
// Function
//====================================================================================================

void MargAHRSupdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float accelCutoff, uint8_t magDataUpdate, float dt)
{
    float norm, normR;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float q0i, q1i, q2i, q3i;

    //-------------------------------------------

    if ((MargAHRSinitialized == false) && (magDataUpdate == true))
    {
        MargAHRSinit(ax, ay, az, mx, my, mz);

        MargAHRSinitialized = true;
    }

    //-------------------------------------------

    if (MargAHRSinitialized == true)
    {
        halfT = dt * 0.5f;

        norm = sqrt(SQR(ax) + SQR(ay) + SQR(az));

        if (norm != 0.0f)
        {
			calculateAccConfidence(norm);
            kpAcc = eepromConfig.KpAcc * accConfidence;
            kiAcc = eepromConfig.KiAcc * accConfidence;

            normR = 1.0f / norm;
            ax *= normR;
            ay *= normR;
            az *= normR;

            // estimated direction of gravity (v)
            vx = 2.0f * (q1q3 - q0q2);
            vy = 2.0f * (q0q1 + q2q3);
            vz = q0q0 - q1q1 - q2q2 + q3q3;

            // error is sum of cross product between reference direction
		    // of fields and direction measured by sensors
		    exAcc = vy * az - vz * ay;
            eyAcc = vz * ax - vx * az;
            ezAcc = vx * ay - vy * ax;

            gx += exAcc * kpAcc;
            gy += eyAcc * kpAcc;
            gz += ezAcc * kpAcc;

            if (kiAcc > 0.0f)
            {
		    	exAccInt += exAcc * kiAcc;
                eyAccInt += eyAcc * kiAcc;
                ezAccInt += ezAcc * kiAcc;

                gx += exAccInt;
                gy += eyAccInt;
                gz += ezAccInt;
		    }
	    }

        //-------------------------------------------

        norm = sqrt(SQR(mx) + SQR(my) + SQR(mz));

        if (( magDataUpdate == true) && (norm != 0.0f))
        {
            normR = 1.0f / norm;
            mx *= normR;
            my *= normR;
            mz *= normR;

            // compute reference direction of flux
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));

            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));

            hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            bx = sqrt((hx * hx) + (hy * hy));

            bz = hz;

            // estimated direction of flux (w)
            wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));

            wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));

            wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

            exMag = my * wz - mz * wy;
            eyMag = mz * wx - mx * wz;
            ezMag = mx * wy - my * wx;

			// use un-extrapolated old values between magnetometer updates
			// dubious as dT does not apply to the magnetometer calculation so
			// time scaling is embedded in KpMag and KiMag
			gx += exMag * eepromConfig.KpMag;
			gy += eyMag * eepromConfig.KpMag;
			gz += ezMag * eepromConfig.KpMag;

			if (eepromConfig.KiMag > 0.0f)
			{
				exMagInt += exMag * eepromConfig.KiMag;
				eyMagInt += eyMag * eepromConfig.KiMag;
				ezMagInt += ezMag * eepromConfig.KiMag;

				gx += exMagInt;
				gy += eyMagInt;
				gz += ezMagInt;
			}
        }

        //-------------------------------------------

        // integrate quaternion rate
        q0i = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
        q1i = ( q0 * gx + q2 * gz - q3 * gy) * halfT;
        q2i = ( q0 * gy - q1 * gz + q3 * gx) * halfT;
        q3i = ( q0 * gz + q1 * gy - q2 * gx) * halfT;
        q0 += q0i;
        q1 += q1i;
        q2 += q2i;
        q3 += q3i;

        // normalise quaternion
        normR = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= normR;
        q1 *= normR;
        q2 *= normR;
        q3 *= normR;

        // auxiliary variables to reduce number of repeated operations
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        sensors.attitude500Hz[ROLL ] = atan2f( 2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3 );
		sensors.attitude500Hz[PITCH] = -asinf( 2.0f * (q1q3 - q0q2) );
		sensors.attitude500Hz[YAW  ] = atan2f( 2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3 );

		heading.mag = sensors.attitude500Hz[YAW];
		heading.tru = standardRadianFormat(heading.mag + eepromConfig.magVar);
    }
}

//====================================================================================================
// END OF CODE
//====================================================================================================
