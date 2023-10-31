// UTM.h

// Original Javascript by Chuck Taylor
// Port to C++ by Alex Hajnal
//
// *** THIS CODE USES 32-BIT FLOATS BY DEFAULT ***
// *** For 64-bit double-precision edit this file: undefine FLOAT_32 and define FLOAT_64 (see below)
//
// This is a simple port of the code on the Geographic/UTM Coordinate Converter (1) page from Javascript to C++.
// Using this you can easily convert between UTM and WGS84 (latitude and longitude).
// Accuracy seems to be around 50cm (I suspect rounding errors are limiting precision).
// This code is provided as-is and has been minimally tested; enjoy but use at your own risk!
// The license for UTM.cpp and UTM.h is the same as the original Javascript:
// "The C++ source code in UTM.cpp and UTM.h may be copied and reused without restriction."
//
// 1) http://home.hiwaay.net/~taylorc/toolbox/geography/geoutm.html

#include <string>
#include <sstream>

#ifndef UTM_H
#define UTM_H

// Choose floating point precision:

// 32-bit (for Teensy 3.5/3.6 ARM boards, etc.)
#define FLOAT_64

// 64-bit (for desktop/server use)
// #define FLOAT_64

#ifdef FLOAT_64
#define FLOAT double
#define SIN sin
#define COS cos
#define TAN tan
#define POW pow
#define SQRT sqrt
#define FLOOR floor

#else
#ifdef FLOAT_32
#define FLOAT float
#define SIN sinf
#define COS cosf
#define TAN tanf
#define POW powf
#define SQRT sqrtf
#define FLOOR floorf

#endif
#endif

#include <math.h>

#define pi 3.14159265358979

/* Ellipsoid model constants (actual values here are for WGS84) */
#define sm_a 6378137.0
#define sm_b 6356752.314
#define sm_EccSquared 6.69437999013e-03

#define UTMScaleFactor 0.9996

// DegToRad
// Converts degrees to radians.
FLOAT DegToRad(FLOAT deg);

// RadToDeg
// Converts radians to degrees.
FLOAT RadToDeg(FLOAT rad);

// ArcLengthOfMeridian
// Computes the ellipsoidal distance from the equator to a point at a
// given latitude.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
// GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//     phi - Latitude of the point, in radians.
//
// Globals:
//     sm_a - Ellipsoid model major axis.
//     sm_b - Ellipsoid model minor axis.
//
// Returns:
//     The ellipsoidal distance of the point from the equator, in meters.
FLOAT ArcLengthOfMeridian(FLOAT phi);

// UTMCentralMeridian
// Determines the central meridian for the given UTM zone.
//
// Inputs:
//     zone - An integer value designating the UTM zone, range [1,60].
//
// Returns:
//   The central meridian for the given UTM zone, in radians
//   Range of the central meridian is the radian equivalent of [-177,+177].
FLOAT UTMCentralMeridian(int zone);

// FootpointLatitude
//
// Computes the footpoint latitude for use in converting transverse
// Mercator coordinates to ellipsoidal coordinates.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
//   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//   y - The UTM northing coordinate, in meters.
//
// Returns:
//   The footpoint latitude, in radians.
FLOAT FootpointLatitude(FLOAT y);

// MapLatLonToXY
// Converts a latitude/longitude pair to x and y coordinates in the
// Transverse Mercator projection.  Note that Transverse Mercator is not
// the same as UTM; a scale factor is required to convert between them.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
// GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//    phi - Latitude of the point, in radians.
//    lambda - Longitude of the point, in radians.
//    lambda0 - Longitude of the central meridian to be used, in radians.
//
// Outputs:
//    x - The x coordinate of the computed point.
//    y - The y coordinate of the computed point.
//
// Returns:
//    The function does not return a value.
void MapLatLonToXY(FLOAT phi, FLOAT lambda, FLOAT lambda0, FLOAT &x, FLOAT &y);

// MapXYToLatLon
// Converts x and y coordinates in the Transverse Mercator projection to
// a latitude/longitude pair.  Note that Transverse Mercator is not
// the same as UTM; a scale factor is required to convert between them.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
//   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//   x - The easting of the point, in meters.
//   y - The northing of the point, in meters.
//   lambda0 - Longitude of the central meridian to be used, in radians.
//
// Outputs:
//   phi    - Latitude in radians.
//   lambda - Longitude in radians.
//
// Returns:
//   The function does not return a value.
//
// Remarks:
//   The local variables Nf, nuf2, tf, and tf2 serve the same purpose as
//   N, nu2, t, and t2 in MapLatLonToXY, but they are computed with respect
//   to the footpoint latitude phif.
//
//   x1frac, x2frac, x2poly, x3poly, etc. are to enhance readability and
//   to optimize computations.
void MapXYToLatLon(FLOAT x, FLOAT y, FLOAT lambda0, FLOAT &phi, FLOAT &lambda);

// LatLonToUTMXY
// Converts a latitude/longitude pair to x and y coordinates in the
// Universal Transverse Mercator projection.
//
// Inputs:
//   lat - Latitude of the point, in radians.
//   lon - Longitude of the point, in radians.
//   zone - UTM zone to be used for calculating values for x and y.
//          If zone is less than 1 or greater than 60, the routine
//          will determine the appropriate zone from the value of lon.
//
// Outputs:
//   x - The x coordinate (easting) of the computed point. (in meters)
//   y - The y coordinate (northing) of the computed point. (in meters)
//
// Returns:
//   The UTM zone used for calculating the values of x and y.
int LatLonToUTMXY(FLOAT lat, FLOAT lon, int zone, FLOAT &x, FLOAT &y);

// UTMXYToLatLon
//
// Converts x and y coordinates in the Universal Transverse Mercator//   The UTM zone parameter should be in the range [1,60].

// projection to a latitude/longitude pair.
//
// Inputs:
// x - The easting of the point, in meters.
// y - The northing of the point, in meters.
// zone - The UTM zone in which the point lies.
// southhemi - True if the point is in the southern hemisphere;
//               false otherwise.
//
// Outputs:
// lat - The latitude of the point, in radians.
// lon - The longitude of the point, in radians.
//
// Returns:
// The function does not return a value.
void UTMXYToLatLon(FLOAT x, FLOAT y, int zone, bool southhemi, FLOAT &lat, FLOAT &lon);

// DegToRad
// Converts degrees to radians.
FLOAT DegToRad(FLOAT deg)
{
  return (deg / 180.0 * pi);
}

// RadToDeg
// Converts radians to degrees.
FLOAT RadToDeg(FLOAT rad)
{
  return (rad / pi * 180.0);
}

// ArcLengthOfMeridian
// Computes the ellipsoidal distance from the equator to a point at a
// given latitude.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
// GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//     phi - Latitude of the point, in radians.
//
// Globals:
//     sm_a - Ellipsoid model major axis.
//     sm_b - Ellipsoid model minor axis.
//
// Returns:
//     The ellipsoidal distance of the point from the equator, in meters.
FLOAT ArcLengthOfMeridian(FLOAT phi)
{
  FLOAT alpha, beta, gamma, delta, epsilon, n;
  FLOAT result;

  /* Precalculate n */
  n = (sm_a - sm_b) / (sm_a + sm_b);

  /* Precalculate alpha */
  alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (POW(n, 2.0) / 4.0) + (POW(n, 4.0) / 64.0));

  /* Precalculate beta */
  beta = (-3.0 * n / 2.0) + (9.0 * POW(n, 3.0) / 16.0) + (-3.0 * POW(n, 5.0) / 32.0);

  /* Precalculate gamma */
  gamma = (15.0 * POW(n, 2.0) / 16.0) + (-15.0 * POW(n, 4.0) / 32.0);

  /* Precalculate delta */
  delta = (-35.0 * POW(n, 3.0) / 48.0) + (105.0 * POW(n, 5.0) / 256.0);

  /* Precalculate epsilon */
  epsilon = (315.0 * POW(n, 4.0) / 512.0);

  /* Now calculate the sum of the series and return */
  result = alpha * (phi + (beta * SIN(2.0 * phi)) + (gamma * SIN(4.0 * phi)) + (delta * SIN(6.0 * phi)) + (epsilon * SIN(8.0 * phi)));

  return result;
}

// UTMCentralMeridian
// Determines the central meridian for the given UTM zone.
//
// Inputs:
//     zone - An integer value designating the UTM zone, range [1,60].
//
// Returns:
//   The central meridian for the given UTM zone, in radians
//   Range of the central meridian is the radian equivalent of [-177,+177].
FLOAT UTMCentralMeridian(int zone)
{
  FLOAT cmeridian;
  cmeridian = DegToRad(-183.0 + ((FLOAT)zone * 6.0));

  return cmeridian;
}

// FootpointLatitude
//
// Computes the footpoint latitude for use in converting transverse
// Mercator coordinates to ellipsoidal coordinates.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
//   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//   y - The UTM northing coordinate, in meters.
//
// Returns:
//   The footpoint latitude, in radians.
FLOAT FootpointLatitude(FLOAT y)
{
  FLOAT y_, alpha_, beta_, gamma_, delta_, epsilon_, n;
  FLOAT result;

  /* Precalculate n (Eq. 10.18) */
  n = (sm_a - sm_b) / (sm_a + sm_b);

  /* Precalculate alpha_ (Eq. 10.22) */
  /* (Same as alpha in Eq. 10.17) */
  alpha_ = ((sm_a + sm_b) / 2.0) * (1 + (POW(n, 2.0) / 4) + (POW(n, 4.0) / 64));

  /* Precalculate y_ (Eq. 10.23) */
  y_ = y / alpha_;

  /* Precalculate beta_ (Eq. 10.22) */
  beta_ = (3.0 * n / 2.0) + (-27.0 * POW(n, 3.0) / 32.0) + (269.0 * POW(n, 5.0) / 512.0);

  /* Precalculate gamma_ (Eq. 10.22) */
  gamma_ = (21.0 * POW(n, 2.0) / 16.0) + (-55.0 * POW(n, 4.0) / 32.0);

  /* Precalculate delta_ (Eq. 10.22) */
  delta_ = (151.0 * POW(n, 3.0) / 96.0) + (-417.0 * POW(n, 5.0) / 128.0);

  /* Precalculate epsilon_ (Eq. 10.22) */
  epsilon_ = (1097.0 * POW(n, 4.0) / 512.0);

  /* Now calculate the sum of the series (Eq. 10.21) */
  result = y_ + (beta_ * SIN(2.0 * y_)) + (gamma_ * SIN(4.0 * y_)) + (delta_ * SIN(6.0 * y_)) + (epsilon_ * SIN(8.0 * y_));

  return result;
}

// MapLatLonToXY
// Converts a latitude/longitude pair to x and y coordinates in the
// Transverse Mercator projection.  Note that Transverse Mercator is not
// the same as UTM; a scale factor is required to convert between them.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
// GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//    phi - Latitude of the point, in radians.
//    lambda - Longitude of the point, in radians.
//    lambda0 - Longitude of the central meridian to be used, in radians.
//
// Outputs:
//    x - The x coordinate of the computed point.
//    y - The y coordinate of the computed point.
//
// Returns:
//    The function does not return a value.
void MapLatLonToXY(FLOAT phi, FLOAT lambda, FLOAT lambda0, FLOAT &x, FLOAT &y)
{
  FLOAT N, nu2, ep2, t, t2, l;
  FLOAT l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;
  // FLOAT tmp; // Unused

  /* Precalculate ep2 */
  ep2 = (POW(sm_a, 2.0) - POW(sm_b, 2.0)) / POW(sm_b, 2.0);

  /* Precalculate nu2 */
  nu2 = ep2 * POW(COS(phi), 2.0);

  /* Precalculate N */
  N = POW(sm_a, 2.0) / (sm_b * SQRT(1 + nu2));

  /* Precalculate t */
  t = TAN(phi);
  t2 = t * t;
  // tmp = (t2 * t2 * t2) - POW(t, 6.0); // Unused

  /* Precalculate l */
  l = lambda - lambda0;

  /* Precalculate coefficients for l**n in the equations below
     so a normal human being can read the expressions for easting
     and northing
     -- l**1 and l**2 have coefficients of 1.0 */
  l3coef = 1.0 - t2 + nu2;

  l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

  l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;

  l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;

  l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

  l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

  /* Calculate easting (x) */
  x = N * COS(phi) * l + (N / 6.0 * POW(COS(phi), 3.0) * l3coef * POW(l, 3.0)) + (N / 120.0 * POW(COS(phi), 5.0) * l5coef * POW(l, 5.0)) + (N / 5040.0 * POW(COS(phi), 7.0) * l7coef * POW(l, 7.0));

  /* Calculate northing (y) */
  y = ArcLengthOfMeridian(phi) + (t / 2.0 * N * POW(COS(phi), 2.0) * POW(l, 2.0)) + (t / 24.0 * N * POW(COS(phi), 4.0) * l4coef * POW(l, 4.0)) + (t / 720.0 * N * POW(COS(phi), 6.0) * l6coef * POW(l, 6.0)) + (t / 40320.0 * N * POW(COS(phi), 8.0) * l8coef * POW(l, 8.0));

  return;
}

// MapXYToLatLon
// Converts x and y coordinates in the Transverse Mercator projection to
// a latitude/longitude pair.  Note that Transverse Mercator is not
// the same as UTM; a scale factor is required to convert between them.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
//   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//   x - The easting of the point, in meters.
//   y - The northing of the point, in meters.
//   lambda0 - Longitude of the central meridian to be used, in radians.
//
// Outputs:
//   phi    - Latitude in radians.
//   lambda - Longitude in radians.
//
// Returns:
//   The function does not return a value.
//
// Remarks:
//   The local variables Nf, nuf2, tf, and tf2 serve the same purpose as
//   N, nu2, t, and t2 in MapLatLonToXY, but they are computed with respect
//   to the footpoint latitude phif.
//
//   x1frac, x2frac, x2poly, x3poly, etc. are to enhance readability and
//   to optimize computations.
void MapXYToLatLon(FLOAT x, FLOAT y, FLOAT lambda0, FLOAT &phi, FLOAT &lambda)
{
  FLOAT phif, Nf, Nfpow, nuf2, ep2, tf, tf2, tf4, cf;
  FLOAT x1frac, x2frac, x3frac, x4frac, x5frac, x6frac, x7frac, x8frac;
  FLOAT x2poly, x3poly, x4poly, x5poly, x6poly, x7poly, x8poly;

  /* Get the value of phif, the footpoint latitude. */
  phif = FootpointLatitude(y);

  /* Precalculate ep2 */
  ep2 = (POW(sm_a, 2.0) - POW(sm_b, 2.0)) / POW(sm_b, 2.0);

  /* Precalculate cos (phif) */
  cf = COS(phif);

  /* Precalculate nuf2 */
  nuf2 = ep2 * POW(cf, 2.0);

  /* Precalculate Nf and initialize Nfpow */
  Nf = POW(sm_a, 2.0) / (sm_b * SQRT(1 + nuf2));
  Nfpow = Nf;

  /* Precalculate tf */
  tf = TAN(phif);
  tf2 = tf * tf;
  tf4 = tf2 * tf2;

  /* Precalculate fractional coefficients for x**n in the equations
     below to simplify the expressions for latitude and longitude. */
  x1frac = 1.0 / (Nfpow * cf);

  Nfpow *= Nf; /* now equals Nf**2) */
  x2frac = tf / (2.0 * Nfpow);

  Nfpow *= Nf; /* now equals Nf**3) */
  x3frac = 1.0 / (6.0 * Nfpow * cf);

  Nfpow *= Nf; /* now equals Nf**4) */
  x4frac = tf / (24.0 * Nfpow);

  Nfpow *= Nf; /* now equals Nf**5) */
  x5frac = 1.0 / (120.0 * Nfpow * cf);

  Nfpow *= Nf; /* now equals Nf**6) */
  x6frac = tf / (720.0 * Nfpow);

  Nfpow *= Nf; /* now equals Nf**7) */
  x7frac = 1.0 / (5040.0 * Nfpow * cf);

  Nfpow *= Nf; /* now equals Nf**8) */
  x8frac = tf / (40320.0 * Nfpow);

  /* Precalculate polynomial coefficients for x**n.
     -- x**1 does not have a polynomial coefficient. */
  x2poly = -1.0 - nuf2;

  x3poly = -1.0 - 2 * tf2 - nuf2;

  x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2 - 3.0 * (nuf2 * nuf2) - 9.0 * tf2 * (nuf2 * nuf2);

  x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;

  x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2 + 162.0 * tf2 * nuf2;

  x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);

  x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);

  /* Calculate latitude */
  phi = phif + x2frac * x2poly * (x * x) + x4frac * x4poly * POW(x, 4.0) + x6frac * x6poly * POW(x, 6.0) + x8frac * x8poly * POW(x, 8.0);

  /* Calculate longitude */
  lambda = lambda0 + x1frac * x + x3frac * x3poly * POW(x, 3.0) + x5frac * x5poly * POW(x, 5.0) + x7frac * x7poly * POW(x, 7.0);

  return;
}

// LatLonToUTMXY
// Converts a latitude/longitude pair to x and y coordinates in the
// Universal Transverse Mercator projection.
//
// Inputs:
//   lat - Latitude of the point, in radians.
//   lon - Longitude of the point, in radians.
//   zone - UTM zone to be used for calculating values for x and y.
//          If zone is less than 1 or greater than 60, the routine
//          will determine the appropriate zone from the value of lon.
//
// Outputs:
//   x - The x coordinate (easting) of the computed point. (in meters)
//   y - The y coordinate (northing) of the computed point. (in meters)
//
// Returns:
//   The UTM zone used for calculating the values of x and y.
int LatLonToUTMXY(FLOAT lat, FLOAT lon, int zone, FLOAT &x, FLOAT &y)
{
  if ((zone < 1) || (zone > 60))
    zone = FLOOR((lon + 180.0) / 6) + 1;

  MapLatLonToXY(DegToRad(lat), DegToRad(lon), UTMCentralMeridian(zone), x, y);

  /* Adjust easting and northing for UTM system. */
  x = x * UTMScaleFactor + 500000.0;
  y = y * UTMScaleFactor;
  if (y < 0.0)
    y = y + 10000000.0;

  return zone;
}

// UTMXYToLatLon
//
// Converts x and y coordinates in the Universal Transverse Mercator
// projection to a latitude/longitude pair.
//
// Inputs:
// x - The easting of the point, in meters.
// y - The northing of the point, in meters.
// zone - The UTM zone in which the point lies.
// southhemi - True if the point is in the southern hemisphere;
//               false otherwise.
//
// Outputs:
// lat - The latitude of the point, in radians.
// lon - The longitude of the point, in radians.
//
// Returns:
// The function does not return a value.
void UTMXYToLatLon(FLOAT x, FLOAT y, int zone, bool southhemi, FLOAT &lat, FLOAT &lon)
{
  FLOAT cmeridian;

  x -= 500000.0;
  x /= UTMScaleFactor;

  /* If in southern hemisphere, adjust y accordingly. */
  if (southhemi)
    y -= 10000000.0;

  y /= UTMScaleFactor;

  cmeridian = UTMCentralMeridian(zone);
  MapXYToLatLon(x, y, cmeridian, lat, lon);

  return;
}

const double WGS84_A = 6378137.0;
const double WGS84_ECCSQ = 0.00669437999013;

char UTMLetterDesignator(double latitude)
{
  // This routine determines the correct UTM letter designator for the given latitude
  // returns 'Z' if latitude is outside the UTM limits of 84N to 80S
  // Written by Chuck Gantz- chuck.gantz@globalstar.com
  char letterDesignator;

  if ((84.0 >= latitude) && (latitude >= 72.0))
    letterDesignator = 'X';
  else if ((72.0 > latitude) && (latitude >= 64.0))
    letterDesignator = 'W';
  else if ((64.0 > latitude) && (latitude >= 56.0))
    letterDesignator = 'V';
  else if ((56.0 > latitude) && (latitude >= 48.0))
    letterDesignator = 'U';
  else if ((48.0 > latitude) && (latitude >= 40.0))
    letterDesignator = 'T';
  else if ((40.0 > latitude) && (latitude >= 32.0))
    letterDesignator = 'S';
  else if ((32.0 > latitude) && (latitude >= 24.0))
    letterDesignator = 'R';
  else if ((24.0 > latitude) && (latitude >= 16.0))
    letterDesignator = 'Q';
  else if ((16.0 > latitude) && (latitude >= 8.0))
    letterDesignator = 'P';
  else if ((8.0 > latitude) && (latitude >= 0.0))
    letterDesignator = 'N';
  else if ((0.0 > latitude) && (latitude >= -8.0))
    letterDesignator = 'M';
  else if ((-8.0 > latitude) && (latitude >= -16.0))
    letterDesignator = 'L';
  else if ((-16.0 > latitude) && (latitude >= -24.0))
    letterDesignator = 'K';
  else if ((-24.0 > latitude) && (latitude >= -32.0))
    letterDesignator = 'J';
  else if ((-32.0 > latitude) && (latitude >= -40.0))
    letterDesignator = 'H';
  else if ((-40.0 > latitude) && (latitude >= -48.0))
    letterDesignator = 'G';
  else if ((-48.0 > latitude) && (latitude >= -56.0))
    letterDesignator = 'F';
  else if ((-56.0 > latitude) && (latitude >= -64.0))
    letterDesignator = 'E';
  else if ((-64.0 > latitude) && (latitude >= -72.0))
    letterDesignator = 'D';
  else if ((-72.0 > latitude) && (latitude >= -80.0))
    letterDesignator = 'C';
  else
    letterDesignator = 'Z'; // This is here as an error flag to show that the Latitude is outside the UTM limits

  return letterDesignator;
}

void LLtoUTM(double latitude, double longitude,
             double &utmNorthing, double &utmEasting, std::string &utmZone)
{
  // converts lat/long to UTM coords.  Equations from USGS Bulletin 1532
  // East Longitudes are positive, West longitudes are negative.
  // North latitudes are positive, South latitudes are negative
  // Lat and Long are in decimal degrees
  // Written by Chuck Gantz- chuck.gantz@globalstar.com

  double k0 = 0.9996;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  double LatRad = latitude * M_PI / 180.0;
  double LongRad = longitude * M_PI / 180.0;
  double LongOriginRad;

  int ZoneNumber = static_cast<int>((longitude + 180.0) / 6.0) + 1;

  if (latitude >= 56.0 && latitude < 64.0 &&
      longitude >= 3.0 && longitude < 12.0)
  {
    ZoneNumber = 32;
  }

  // Special zones for Svalbard
  if (latitude >= 72.0 && latitude < 84.0)
  {
    if (longitude >= 0.0 && longitude < 9.0)
      ZoneNumber = 31;
    else if (longitude >= 9.0 && longitude < 21.0)
      ZoneNumber = 33;
    else if (longitude >= 21.0 && longitude < 33.0)
      ZoneNumber = 35;
    else if (longitude >= 33.0 && longitude < 42.0)
      ZoneNumber = 37;
  }
  LongOrigin = static_cast<double>((ZoneNumber - 1) * 6 - 180 + 3); //+3 puts origin in middle of zone
  LongOriginRad = LongOrigin * M_PI / 180.0;

  // compute the UTM Zone from the latitude and longitude
  std::ostringstream oss;
  oss << ZoneNumber << UTMLetterDesignator(latitude);
  utmZone = oss.str();

  eccPrimeSquared = WGS84_ECCSQ / (1.0 - WGS84_ECCSQ);

  N = WGS84_A / sqrt(1.0 - WGS84_ECCSQ * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad - LongOriginRad);

  M = WGS84_A * ((1.0 - WGS84_ECCSQ / 4.0 - 3.0 * WGS84_ECCSQ * WGS84_ECCSQ / 64.0 - 5.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 256.0) * LatRad - (3.0 * WGS84_ECCSQ / 8.0 + 3.0 * WGS84_ECCSQ * WGS84_ECCSQ / 32.0 + 45.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 1024.0) * sin(2.0 * LatRad) + (15.0 * WGS84_ECCSQ * WGS84_ECCSQ / 256.0 + 45.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 1024.0) * sin(4.0 * LatRad) - (35.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 3072.0) * sin(6.0 * LatRad));

  utmEasting = k0 * N * (A + (1.0 - T + C) * A * A * A / 6.0 + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * eccPrimeSquared) * A * A * A * A * A / 120.0) + 500000.0;

  utmNorthing = k0 * (M + N * tan(LatRad) *
                              (A * A / 2.0 +
                               (5.0 - T + 9.0 * C + 4.0 * C * C) * A * A * A * A / 24.0 + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * eccPrimeSquared) * A * A * A * A * A * A / 720.0));
  if (latitude < 0.0)
  {
    utmNorthing += 10000000.0; // 10000000 meter offset for southern hemisphere
  }
}

void UTMtoLL(double utmNorthing, double utmEasting, const std::string &utmZone,
             double &latitude, double &longitude)
{
  // converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
  // East Longitudes are positive, West longitudes are negative.
  // North latitudes are positive, South latitudes are negative
  // Lat and Long are in decimal degrees.
  // Written by Chuck Gantz- chuck.gantz@globalstar.com

  double k0 = 0.9996;
  double eccPrimeSquared;
  double e1 = (1.0 - sqrt(1.0 - WGS84_ECCSQ)) / (1.0 + sqrt(1.0 - WGS84_ECCSQ));
  double N1, T1, C1, R1, D, M;
  double LongOrigin;
  double mu, phi1, phi1Rad;
  double x, y;
  int ZoneNumber;
  char ZoneLetter;
  bool NorthernHemisphere;

  x = utmEasting - 500000.0; // remove 500,000 meter offset for longitude
  y = utmNorthing;

  std::istringstream iss(utmZone);
  iss >> ZoneNumber >> ZoneLetter;
  if ((static_cast<int>(ZoneLetter) - static_cast<int>('N')) >= 0)
  {
    NorthernHemisphere = true; // point is in northern hemisphere
  }
  else
  {
    NorthernHemisphere = false; // point is in southern hemisphere
    y -= 10000000.0;            // remove 10,000,000 meter offset used for southern hemisphere
  }

  LongOrigin = (ZoneNumber - 1.0) * 6.0 - 180.0 + 3.0; //+3 puts origin in middle of zone

  eccPrimeSquared = WGS84_ECCSQ / (1.0 - WGS84_ECCSQ);

  M = y / k0;
  mu = M / (WGS84_A * (1.0 - WGS84_ECCSQ / 4.0 - 3.0 * WGS84_ECCSQ * WGS84_ECCSQ / 64.0 - 5.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 256.0));

  phi1Rad = mu + (3.0 * e1 / 2.0 - 27.0 * e1 * e1 * e1 / 32.0) * sin(2.0 * mu) + (21.0 * e1 * e1 / 16.0 - 55.0 * e1 * e1 * e1 * e1 / 32.0) * sin(4.0 * mu) + (151.0 * e1 * e1 * e1 / 96.0) * sin(6.0 * mu);
  phi1 = phi1Rad / M_PI * 180.0;

  N1 = WGS84_A / sqrt(1.0 - WGS84_ECCSQ * sin(phi1Rad) * sin(phi1Rad));
  T1 = tan(phi1Rad) * tan(phi1Rad);
  C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
  R1 = WGS84_A * (1.0 - WGS84_ECCSQ) /
       pow(1.0 - WGS84_ECCSQ * sin(phi1Rad) * sin(phi1Rad), 1.5);
  D = x / (N1 * k0);

  latitude = phi1Rad - (N1 * tan(phi1Rad) / R1) * (D * D / 2.0 - (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1 - 9.0 * eccPrimeSquared) * D * D * D * D / 24.0 + (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1 - 252.0 * eccPrimeSquared - 3.0 * C1 * C1) * D * D * D * D * D * D / 720.0);
  latitude *= 180.0 / M_PI;

  longitude = (D - (1.0 + 2.0 * T1 + C1) * D * D * D / 6.0 + (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1 + 8.0 * eccPrimeSquared + 24.0 * T1 * T1) * D * D * D * D * D / 120.0) / cos(phi1Rad);
  longitude = LongOrigin + longitude / M_PI * 180.0;
}

#endif
