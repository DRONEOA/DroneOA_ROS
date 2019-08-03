#ifndef GPS_POINT_
#define GPS_POINT_

class GPSPoint {
public:
    GPSPoint();
    GPSPoint(float latitude, float longitude, float altitude);
    virtual ~GPSPoint() = default;

    float latitude_;
    float longitude_;
    float altitude_;
};

#endif