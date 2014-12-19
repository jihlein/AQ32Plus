///////////////////////////////////////////////////////////////////////////////

#include   "board.h"
//#include "wmm2010.h"
#include   "wmm2015.h"

///////////////////////////////////////////////////////////////////////////////

MAGtype_CoordGeodetic coordGeodetic;

MAGtype_CoordSpherical coordSpherical;

MAGtype_Date userDate;

MAGtype_Ellipsoid ellipsoid;

MAGtype_GeoMagneticElements geoMagneticElements;

MAGtype_MagneticModel *timedMagneticModel, *magneticModel;

///////////////////////////////////////////////////////////////////////////////

void setupGeoMagWorkspace(void)
{
    uint8_t  index;
    uint16_t nMax = 12;
    uint16_t numTerms;

    ///////////////////////////////////

    numTerms = CALCULATE_NUMTERMS(nMax);

    magneticModel = MAG_AllocateModelMemory(numTerms);

    ///////////////////////////////////

    magneticModel->nMax       = nMax;
	magneticModel->nMaxSecVar = nMax;
    magneticModel->epoch      = 2010.0f;

    strncpy(magneticModel->ModelName, "WMM-2015        12/15/2014\0", sizeof(magneticModel->ModelName));

    magneticModel->Main_Field_Coeff_G[0]  = 0.0f;
    magneticModel->Main_Field_Coeff_H[0]  = 0.0f;
    magneticModel->Secular_Var_Coeff_G[0] = 0.0f;
    magneticModel->Secular_Var_Coeff_H[0] = 0.0f;

    for ( index = 1; index < 91; index++)
    {
		magneticModel->Main_Field_Coeff_G[index]  = wmmCoefficients[index - 1][0];  // gnm
		magneticModel->Main_Field_Coeff_H[index]  = wmmCoefficients[index - 1][1];  // hnm
        magneticModel->Secular_Var_Coeff_G[index] = wmmCoefficients[index - 1][2];  // dgnm
		magneticModel->Secular_Var_Coeff_H[index] = wmmCoefficients[index - 1][3];  // dhnm
	}

	magneticModel->CoefficientFileEndDate = magneticModel->epoch + 5.0f;

	///////////////////////////////////

	numTerms = ((nMax + 1) * (nMax + 2) / 2);

    timedMagneticModel = MAG_AllocateModelMemory(numTerms);

    ///////////////////////////////////

    //  Set WGS-84 parameters
	ellipsoid.a     = 6378.137f;                                                               // semi-major axis of the ellipsoid in km
	ellipsoid.b     = 6356.7523142f;                                                           // semi-minor axis of the ellipsoid in km
	ellipsoid.fla   = 1.0f / 298.257223563f;                                                   // flattening
	ellipsoid.eps   = sqrt(1.0f - (ellipsoid.b * ellipsoid.b) / (ellipsoid.a * ellipsoid.a));  // first eccentricity
	ellipsoid.epssq = (ellipsoid.eps * ellipsoid.eps);                                         // first eccentricity squared
    ellipsoid.re    = 6371.2f;                                                                 // earth's radius in km

    ///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////

void clearGeoMagWorkspace(void)
{
	MAG_FreeMagneticModelMemory(magneticModel);
	MAG_FreeMagneticModelMemory(timedMagneticModel);
}

///////////////////////////////////////////////////////////////////////////////

void computeGeoMagElements(void)
{
	setupGeoMagWorkspace();

	coordGeodetic.lambda               = (float)gps.longitude * 1E-7f;  // Convert to decimal degrees
	coordGeodetic.phi                  = (float)gps.latitude  * 1E-7f;  // Convert to decimal degrees
	coordGeodetic.HeightAboveEllipsoid = (float)gps.height    * 1E-6f;  // Convert from mm to km
	coordGeodetic.UseGeoid             = 0;

	userDate.Year  = gps.year;
	userDate.Month = gps.month;
	userDate.Day   = gps.day;

	//cliPortPrintF("GPS Lat: %7.3f\n", coordGeodetic.phi);
	//cliPortPrintF("GPS Lon: %7.3f\n", coordGeodetic.lambda);
    //cliPortPrintF("GPS Alt: %7.3f\n", coordGeodetic.HeightAboveEllipsoid);
    //cliPortPrint ("\n");

    //MAG_DateToYear(&userDate);

	//cliPortPrintF("GPS Mon:     %4ld\n",  userDate.Month);
    //cliPortPrintF("GPS Day:     %4ld\n",  userDate.Day);
    //cliPortPrintF("GPS Yr:      %4ld\n",  userDate.Year);
    //cliPortPrintF("GPS Dec Yr:  %7.3f\n", userDate.DecimalYear);
    //cliPortPrint ("\n");

	userDate.DecimalYear               = readFloatCLI();
	coordGeodetic.HeightAboveEllipsoid = readFloatCLI();
	coordGeodetic.phi                  = readFloatCLI();
	coordGeodetic.lambda               = readFloatCLI();

	MAG_GeodeticToSpherical(ellipsoid, coordGeodetic, &coordSpherical);

	MAG_TimelyModifyMagneticModel(userDate, magneticModel, timedMagneticModel);

	MAG_Geomag(ellipsoid, coordSpherical, coordGeodetic, timedMagneticModel, &geoMagneticElements);

	cliPortPrint ("\n");
	cliPortPrintF("Dec Yr: %8.1f\n", userDate.DecimalYear);
    cliPortPrintF("Alt:    %8.1f\n", coordGeodetic.HeightAboveEllipsoid);
    cliPortPrintF("Lat:    %8.1f\n", coordGeodetic.phi);
	cliPortPrintF("Lon:    %8.1f\n", coordGeodetic.lambda);
    cliPortPrint ("\n");
    cliPortPrintF("X:      %8.1f\n", geoMagneticElements.X);
	cliPortPrintF("Y:      %8.1f\n", geoMagneticElements.Y);
	cliPortPrintF("Z:      %8.1f\n", geoMagneticElements.Z);
	cliPortPrintF("H:      %8.1f\n", geoMagneticElements.H);
	cliPortPrintF("F:      %8.1f\n", geoMagneticElements.F);
	cliPortPrintF("INCL:   %8.1f\n", geoMagneticElements.Incl);
	cliPortPrintF("DECL:   %8.1f\n", geoMagneticElements.Decl);
	cliPortPrint ("\n");

	clearGeoMagWorkspace();
}

///////////////////////////////////////////////////////////////////////////////
