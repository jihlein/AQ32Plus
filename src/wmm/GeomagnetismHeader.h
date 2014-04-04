/*	WMM Subroutine library was tested in the following environments
 *
 *	1. Red Hat Linux  with GCC Compiler
 *	2. MS Windows XP with CodeGear C++ compiler
 *	3. Sun Solaris with GCC Compiler
 *
 *
 *      Revision Number: $Revision: 833 $
 *      Last changed by: $Author: awoods $
 *      Last changed on: $Date: 2012-04-16 14:01:51 -0600 (Mon, 16 Apr 2012) $
 *
 *
 */


#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE
#endif

//#ifndef EPOCHRANGE
//#define EPOCHRANGE (int)5
//#endif

#define READONLYMODE "r"
#define MAXLINELENGTH (1024)
#define NOOFPARAMS (15)
#define NOOFCOEFFICIENTS (7)


#define _DEGREE_NOT_FOUND (-2)
#define CALCULATE_NUMTERMS(N)    (N * ( N + 1 ) / 2 + N)

#ifndef WMMHEADER_H
#define WMMHEADER_H

#ifndef M_PI
#define M_PI    ((2)*(acos(0.0)))
#endif

#define RAD2DEG(rad)    ((rad)*(180.0L/M_PI))
#define DEG2RAD(deg)    ((deg)*(M_PI/180.0L))
#define ATanH(x)	    (0.5 * log((1 + x) / (1 - x)))


#define TRUE            ((int)1)
#define FALSE           ((int)0)




#define MAG_PS_MIN_LAT_DEGREE  -55 /* Minimum Latitude for  Polar Stereographic projection in degrees   */
#define MAG_PS_MAX_LAT_DEGREE  55  /* Maximum Latitude for Polar Stereographic projection in degrees     */
#define MAG_UTM_MIN_LAT_DEGREE -80.5  /* Minimum Latitude for UTM projection in degrees   */
#define MAG_UTM_MAX_LAT_DEGREE  84.5  /* Maximum Latitude for UTM projection in degrees     */

#define MAG_GEO_POLE_TOLERANCE  1e-5
#define MAG_USE_GEOID	1    /* 1 Geoid - Ellipsoid difference should be corrected, 0 otherwise */

/*
Data types and prototype declaration for
World Magnetic Model (WMM) subroutines.

July 28, 2009

manoj.c.nair@noaa.gov*/


typedef struct {
    float EditionDate;
    float epoch; //Base time of Geomagnetic model epoch (yrs)
    char ModelName[32];
    float *Main_Field_Coeff_G; // C - Gauss coefficients of main geomagnetic model (nT)
    float *Main_Field_Coeff_H; // C - Gauss coefficients of main geomagnetic model (nT)
    float *Secular_Var_Coeff_G; // CD - Gauss coefficients of secular geomagnetic model (nT/yr)
    float *Secular_Var_Coeff_H; // CD - Gauss coefficients of secular geomagnetic model (nT/yr)
    int nMax; // Maximum degree of spherical harmonic model
    int nMaxSecVar; //Maxumum degree of spherical harmonic secular model
    int SecularVariationUsed; //Whether or not the magnetic secular variation vector will be needed by program
    float CoefficientFileEndDate; //
} MAGtype_MagneticModel;

typedef struct {
    float a; /*semi-major axis of the ellipsoid*/
    float b; /*semi-minor axis of the ellipsoid*/
    float fla; /* flattening */
    float epssq; /*first eccentricity squared */
    float eps; /* first eccentricity */
    float re; /* mean radius of  ellipsoid*/
} MAGtype_Ellipsoid;

typedef struct {
    float lambda; // longitude
    float phi; // geodetic latitude
    float HeightAboveEllipsoid; // height above the ellipsoid (HaE)
    float HeightAboveGeoid; /* (height above the EGM96 geoid model ) */
    int UseGeoid;
} MAGtype_CoordGeodetic;

typedef struct {
    float lambda; /* longitude*/
    float phig; /* geocentric latitude*/
    float r; /* distance from the center of the ellipsoid*/
} MAGtype_CoordSpherical;

typedef struct {
    int Year;
    int Month;
    int Day;
    float DecimalYear; /* decimal years */
} MAGtype_Date;

/*typedef struct {
    int MAG_Mercator;
    int MAG_LambertConformalConic;
    int MAG_PolarStereographic;
    int MAG_TransverseMercator;
} MAGtype_MapProjectionCode;
 Deprecated, Will be removed in the next revision.*/

typedef struct {
    float *Pcup; /* Legendre Function */
    float *dPcup; /* Derivative of Legendre fcn */
} MAGtype_LegendreFunction;

typedef struct {
    float Bx; /* North */
    float By; /* East */
    float Bz; /* Down */
} MAGtype_MagneticResults;

typedef struct {
    float *RelativeRadiusPower; /* [earth_reference_radius_km / sph. radius ]^n  */
    float *cos_mlambda; /*cp(m)  - cosine of (m*spherical coord. longitude)*/
    float *sin_mlambda; /* sp(m)  - sine of (m*spherical coord. longitude) */
} MAGtype_SphericalHarmonicVariables;

typedef struct {
    float Decl; /* 1. Angle between the magnetic field vector and true north, positive east*/
    float Incl; /*2. Angle between the magnetic field vector and the horizontal plane, positive down*/
    float F; /*3. Magnetic Field Strength*/
    float H; /*4. Horizontal Magnetic Field Strength*/
    float X; /*5. Northern component of the magnetic field vector*/
    float Y; /*6. Eastern component of the magnetic field vector*/
    float Z; /*7. Downward component of the magnetic field vector*/
    float GV; /*8. The Grid Variation*/
    float Decldot; /*9. Yearly Rate of change in declination*/
    float Incldot; /*10. Yearly Rate of change in inclination*/
    float Fdot; /*11. Yearly rate of change in Magnetic field strength*/
    float Hdot; /*12. Yearly rate of change in horizontal field strength*/
    float Xdot; /*13. Yearly rate of change in the northern component*/
    float Ydot; /*14. Yearly rate of change in the eastern component*/
    float Zdot; /*15. Yearly rate of change in the downward component*/
    float GVdot; /*16. Yearly rate of change in grid variation*/
} MAGtype_GeoMagneticElements;

typedef struct {
    int NumbGeoidCols; /* 360 degrees of longitude at 15 minute spacing */
    int NumbGeoidRows; /* 180 degrees of latitude  at 15 minute spacing */
    int NumbHeaderItems; /* min, max lat, min, max long, lat, long spacing*/
    int ScaleFactor; /* 4 grid cells per degree at 15 minute spacing  */
    float *GeoidHeightBuffer;
    int NumbGeoidElevs;
    int Geoid_Initialized; /* indicates successful initialization */
    int UseGeoid; /*Is the Geoid being used?*/
} MAGtype_Geoid;

typedef struct {
    char Longitude[40];
    char Latitude[40];
} MAGtype_CoordGeodeticStr;

typedef struct {
    float Easting; /* (X) in meters*/
    float Northing; /* (Y) in meters */
    int Zone; /*UTM Zone*/
    char HemiSphere;
    float CentralMeridian;
    float ConvergenceOfMeridians;
    float PointScale;
} MAGtype_UTMParameters;

enum PARAMS {
    SHDF,
    MODELNAME,
    PUBLISHER,
    RELEASEDATE,
    DATACUTOFF,
    MODELSTARTYEAR,
    MODELENDYEAR,
    EPOCH,
    INTSTATICDEG,
    INTSECVARDEG,
    EXTSTATICDEG,
    EXTSECVARDEG,
    GEOMAGREFRAD,
    NORMALIZATION,
    SPATBASFUNC
};

enum COEFFICIENTS {
    IE,
    N,
    M,
    GNM,
    HNM,
    DGNM,
    DHNM
};

enum YYYYMMDD {
    YEAR,
    MONTH,
    DAY
};

/*Prototypes */

/*Wrapper Functions*/
int MAG_Geomag(MAGtype_Ellipsoid Ellip,
        MAGtype_CoordSpherical CoordSpherical,
        MAGtype_CoordGeodetic CoordGeodetic,
        MAGtype_MagneticModel *TimedMagneticModel,
        MAGtype_GeoMagneticElements *GeoMagneticElements);

int MAG_Grid(MAGtype_CoordGeodetic minimum,
        MAGtype_CoordGeodetic maximum,
        float step_size,
        float altitude_step_size,
        float time_step,
        MAGtype_MagneticModel *MagneticModel,
        MAGtype_Geoid *geoid,
        MAGtype_Ellipsoid Ellip,
        MAGtype_Date StartDate,
        MAGtype_Date EndDate,
        int ElementOption,
        int PrintOption,
        char *OutputFile);

int MAG_robustReadMagneticModel_Large(char *filename, char* filenameSV, MAGtype_MagneticModel **MagneticModel, int array_size);

int MAG_robustReadMagModels(char *filename, MAGtype_MagneticModel *(*magneticmodels)[], int array_size);

int MAG_SetDefaults(MAGtype_Ellipsoid *Ellip, MAGtype_Geoid *Geoid);

/*User Interface*/

void MAG_Error(int control);

char MAG_GeomagIntroduction_WMM(MAGtype_MagneticModel *MagneticModel, char *VersionDate);

char MAG_GeomagIntroduction_EMM(MAGtype_MagneticModel *MagneticModel, char *VersionDate);

int MAG_GetUserGrid(MAGtype_CoordGeodetic *minimum,
        MAGtype_CoordGeodetic *maximum,
        float *step_size,
        float *a_step_size,
        float *step_time,
        MAGtype_Date *StartDate,
        MAGtype_Date *EndDate,
        int *ElementOption,
        int *PrintOption,
        char *OutputFile,
        MAGtype_Geoid *Geoid);

int MAG_GetUserInput(MAGtype_MagneticModel *MagneticModel,
        MAGtype_Geoid *Geoid,
        MAGtype_CoordGeodetic *CoordGeodetic,
        MAGtype_Date *MagneticDate);

void MAG_PrintUserData(MAGtype_GeoMagneticElements GeomagElements,
        MAGtype_CoordGeodetic SpaceInput,
        MAGtype_Date TimeInput,
        MAGtype_MagneticModel *MagneticModel,
        MAGtype_Geoid *Geoid);

int MAG_ValidateDMSstringlat(char *input, char *Error);

int MAG_ValidateDMSstringlong(char *input, char *Error);

int MAG_Warnings(int control, float value, MAGtype_MagneticModel *MagneticModel);

/*Memory and File Processing*/

MAGtype_LegendreFunction *MAG_AllocateLegendreFunctionMemory(int NumTerms);

MAGtype_MagneticModel *MAG_AllocateModelMemory(int NumTerms);

MAGtype_SphericalHarmonicVariables *MAG_AllocateSphVarMemory(int nMax);

void MAG_AssignHeaderValues(MAGtype_MagneticModel *model, char values[][MAXLINELENGTH]);

void MAG_AssignMagneticModelCoeffs(MAGtype_MagneticModel *Assignee, MAGtype_MagneticModel *Source, int nMax, int nMaxSecVar);

int MAG_FreeMemory(MAGtype_MagneticModel *MagneticModel, MAGtype_MagneticModel *TimedMagneticModel, MAGtype_LegendreFunction *LegendreFunction);

int MAG_FreeLegendreMemory(MAGtype_LegendreFunction *LegendreFunction);

int MAG_FreeMagneticModelMemory(MAGtype_MagneticModel *MagneticModel);

int MAG_FreeSphVarMemory(MAGtype_SphericalHarmonicVariables *SphVar);

void MAG_PrintWMMFormat(char *filename, MAGtype_MagneticModel *MagneticModel);

void MAG_PrintEMMFormat(char *filename, char *filenameSV, MAGtype_MagneticModel *MagneticModel);

int MAG_readMagneticModel(char *filename, MAGtype_MagneticModel *MagneticModel);

int MAG_readMagneticModel_Large(char *filename, char *filenameSV, MAGtype_MagneticModel *MagneticModel);

int MAG_readMagneticModel_SHDF(char *filename, MAGtype_MagneticModel *(*magneticmodels)[], int array_size);

int MAG_swab_type(void);  // HJI int MAG_swab_type();

char *MAG_Trim(char *str);

float MAG_FloatSwap(float f);

/*Conversions, Transformations, and other Calculations*/

int MAG_CalculateGeoMagneticElements(MAGtype_MagneticResults *MagneticResultsGeo, MAGtype_GeoMagneticElements *GeoMagneticElements);

int MAG_CalculateSecularVariationElements(MAGtype_MagneticResults MagneticVariation, MAGtype_GeoMagneticElements *MagneticElements);

int MAG_CalculateGridVariation(MAGtype_CoordGeodetic location, MAGtype_GeoMagneticElements *elements);

int MAG_DateToYear(MAGtype_Date *Calendar_Date);  // HJI int MAG_DateToYear(MAGtype_Date *Calendar_Date, char *Error);

void MAG_DegreeToDMSstring(float DegreesOfArc, int UnitDepth, char *DMSstring);

void MAG_DMSstringToDegree(char *DMSstring, float *DegreesOfArc);

int MAG_GeodeticToSpherical(MAGtype_Ellipsoid Ellip, MAGtype_CoordGeodetic CoordGeodetic, MAGtype_CoordSpherical *CoordSpherical);

int MAG_GetTransverseMercator(MAGtype_CoordGeodetic CoordGeodetic, MAGtype_UTMParameters *UTMParameters);

int MAG_GetUTMParameters(float Latitude,
        float Longitude,
        int *Zone,
        char *Hemisphere,
        float *CentralMeridian);

int MAG_isNaN(float d);

int MAG_RotateMagneticVector(MAGtype_CoordSpherical,
        MAGtype_CoordGeodetic CoordGeodetic,
        MAGtype_MagneticResults MagneticResultsSph,
        MAGtype_MagneticResults *MagneticResultsGeo);

void MAG_SphericalToCartesian(MAGtype_CoordSpherical CoordSpherical, float *x, float *y, float *z);

void MAG_TMfwd4(float Eps, float Epssq, float K0R4, float K0R4oa,
        float Acoeff[], float Lam0, float K0, float falseE,
        float falseN, int XYonly, float Lambda, float Phi,
        float *X, float *Y, float *pscale, float *CoM);  //Rewrite this?

int MAG_YearToDate(MAGtype_Date *Date);


/*Spherical Harmonics*/

int MAG_AssociatedLegendreFunction(MAGtype_CoordSpherical CoordSpherical, int nMax, MAGtype_LegendreFunction *LegendreFunction);

int MAG_CheckGeographicPole(MAGtype_CoordGeodetic *CoordGeodetic);

int MAG_ComputeSphericalHarmonicVariables(MAGtype_Ellipsoid Ellip,
        MAGtype_CoordSpherical CoordSpherical,
        int nMax,
        MAGtype_SphericalHarmonicVariables * SphVariables);

int MAG_PcupHigh(float *Pcup, float *dPcup, float x, int nMax);

int MAG_PcupLow(float *Pcup, float *dPcup, float x, int nMax);

int MAG_SecVarSummation(MAGtype_LegendreFunction *LegendreFunction,
        MAGtype_MagneticModel *MagneticModel,
        MAGtype_SphericalHarmonicVariables SphVariables,
        MAGtype_CoordSpherical CoordSpherical,
        MAGtype_MagneticResults *MagneticResults);

int MAG_SecVarSummationSpecial(MAGtype_MagneticModel *MagneticModel,
        MAGtype_SphericalHarmonicVariables SphVariables,
        MAGtype_CoordSpherical CoordSpherical,
        MAGtype_MagneticResults *MagneticResults);

int MAG_Summation(MAGtype_LegendreFunction *LegendreFunction,
        MAGtype_MagneticModel *MagneticModel,
        MAGtype_SphericalHarmonicVariables SphVariables,
        MAGtype_CoordSpherical CoordSpherical,
        MAGtype_MagneticResults *MagneticResults);

int MAG_SummationSpecial(MAGtype_MagneticModel *MagneticModel,
        MAGtype_SphericalHarmonicVariables SphVariables,
        MAGtype_CoordSpherical CoordSpherical,
        MAGtype_MagneticResults *MagneticResults);

int MAG_TimelyModifyMagneticModel(MAGtype_Date UserDate, MAGtype_MagneticModel *MagneticModel, MAGtype_MagneticModel *TimedMagneticModel);

/*Geoid*/

int MAG_InitializeGeoid(MAGtype_Geoid *Geoid);



int MAG_ConvertGeoidToEllipsoidHeight(MAGtype_CoordGeodetic *CoordGeodetic, MAGtype_Geoid *Geoid);
/*
 * The function Convert_Geoid_To_Ellipsoid_Height converts the specified WGS84
 * geoid height at the specified geodetic coordinates to the equivalent
 * ellipsoid height, using the EGM96 gravity model.
 *
 *    Latitude            : Geodetic latitude in radians           (input)
 *    Longitude           : Geodetic longitude in radians          (input)
 *    Geoid_Height        : Geoid height, in meters                (input)
 *    Ellipsoid_Height    : Ellipsoid height, in meters.           (output)
 *
 */

int MAG_GetGeoidHeight(float Latitude, float Longitude, float *DeltaHeight, MAGtype_Geoid *Geoid);
/*
 * The private function Get_Geoid_Height returns the height of the
 * WGS84 geiod above or below the WGS84 ellipsoid,
 * at the specified geodetic coordinates,
 * using a grid of height adjustments from the EGM96 gravity model.
 *
 *    Latitude            : Geodetic latitude in radians           (input)
 *    Longitude           : Geodetic longitude in radians          (input)
 *    DeltaHeight         : Height Adjustment, in meters.          (output)
 *
 */






#endif /*WMMHEADER_H*/
