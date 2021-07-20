#ifndef UTM_TRAN_H
#define UTM_TRAN_H

#include <string>
#include <cmath>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <memory>

// WGS84 Parameters
#define WGS84_A 6378137.0        // major axis
#define WGS84_B 6356752.31424518 // minor axis
#define WGS84_F 0.0033528107     // ellipsoid flattening
#define WGS84_E 0.0818191908     // first eccentricity
#define WGS84_EP 0.0820944379    // second eccentricity

// UTM Parameters
#define UTM_K0 0.9996                   // scale factor
#define UTM_FE 500000.0                 // false easting
#define UTM_FN_N 0.0                    // false northing, northern hemisphere
#define UTM_FN_S 10000000.0             // false northing, southern hemisphere
#define UTM_E2 (WGS84_E * WGS84_E)      // e^2
#define UTM_E4 (UTM_E2 * UTM_E2)        // e^4
#define UTM_E6 (UTM_E4 * UTM_E2)        // e^6
#define UTM_EP2 (UTM_E2 / (1 - UTM_E2)) // e'^2
// Grid granularity for rounding UTM coordinates to generate MapXY.
#define grid_size 100000.0 // 100 km grid
#define PI 3.141592653589793
#define RADIANS_PER_DEGREE 0.017453292519943294
#define DEGREES_PER_RADIAN 57.29577951308232522583

namespace tool_t
{
    class UTMTransform
    {
    public:
        ~UTMTransform();
        static std::shared_ptr<UTMTransform> instance(void);

        void UTMtoLL(const double UTMNorthing, const double UTMEasting, double& Lat, double& Long);
        double LLToUTM(const double Lat, const double Long, double& UTMEasting, double& UTMNorthing);
        char UTMLetterDesignator(double Lat);

    private:
        UTMTransform(const UTMTransform&) {}
        UTMTransform();
        std::string UTMZone;
    };
}
#endif