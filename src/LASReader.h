#ifndef LASREADER_H
#define LASREADER_H

#include "PointCloudData.h"
#include <string>
#include <fstream>
#include <cstdint>

#pragma pack(push, 1)
struct LASHeader
{
    char signature[4];
    uint16_t fileSourceID;
    uint16_t globalEncoding;
    uint32_t projectID1;
    uint16_t projectID2;
    uint16_t projectID3;
    uint8_t projectID4[8];
    uint8_t versionMajor;
    uint8_t versionMinor;
    char systemIdentifier[32];
    char generatingSoftware[32];
    uint16_t fileCreationDay;
    uint16_t fileCreationYear;
    uint16_t headerSize;
    uint32_t offsetToPointData;
    uint32_t numberOfVariableLengthRecords;
    uint8_t pointDataRecordFormat;
    uint16_t pointDataRecordLength;
    uint32_t legacyNumberOfPointRecords;
    uint32_t legacyNumberOfPointsByReturn[5];
    double xScaleFactor;
    double yScaleFactor;
    double zScaleFactor;
    double xOffset;
    double yOffset;
    double zOffset;
    double maxX;
    double minX;
    double maxY;
    double minY;
    double maxZ;
    double minZ;
};

struct LASPointFormat0
{
    int32_t x;
    int32_t y;
    int32_t z;
    uint16_t intensity;
    uint8_t returnNumberAndNumberOfReturns;
    uint8_t scanDirectionFlagAndEdgeOfFlightLine;
    uint8_t classification;
    int8_t scanAngleRank;
    uint8_t userData;
    uint16_t pointSourceID;
};

struct LASPointFormat1 : public LASPointFormat0
{
    double gpsTime;
};

struct LASPointFormat2 : public LASPointFormat0
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};

struct LASPointFormat3 : public LASPointFormat1
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};
#pragma pack(pop)

class LASReader
{
public:
    LASReader();
    ~LASReader();
    
    bool read(const std::string& filename, PointCloudData& pointCloud);
    
    std::string getLastError() const { return m_lastError; }

private:
    bool validateHeader(const LASHeader& header);
    template<typename PointType>
    bool readPoints(std::ifstream& file, PointCloudData& pointCloud, 
                    const LASHeader& header, size_t points);
    
    std::string m_lastError;
};

#endif // LASREADER_H
