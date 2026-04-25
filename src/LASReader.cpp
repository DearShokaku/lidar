#include "LASReader.h"
#include <iostream>
#include <cstring>

LASReader::LASReader()
{
}

LASReader::~LASReader()
{
}

bool LASReader::read(const std::string& filename, PointCloudData& pointCloud)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        m_lastError = "Cannot open file: " + filename;
        return false;
    }
    
    pointCloud.clear();
    
    LASHeader header;
    if (!file.read(reinterpret_cast<char*>(&header), sizeof(LASHeader)))
    {
        m_lastError = "Cannot read LAS header";
        return false;
    }
    
    if (!validateHeader(header))
    {
        return false;
    }
    
    file.seekg(header.offsetToPointData, std::ios::beg);
    if (!file.good())
    {
        m_lastError = "Cannot seek to point data";
        return false;
    }
    
    size_t points = header.legacyNumberOfPointRecords;
    
    switch (header.pointDataRecordFormat)
    {
        case 0:
            return readPoints<LASPointFormat0>(file, pointCloud, header, points);
        case 1:
            return readPoints<LASPointFormat1>(file, pointCloud, header, points);
        case 2:
            return readPoints<LASPointFormat2>(file, pointCloud, header, points);
        case 3:
            return readPoints<LASPointFormat3>(file, pointCloud, header, points);
        default:
            m_lastError = "Unsupported point data format: " + 
                         std::to_string(header.pointDataRecordFormat);
            return false;
    }
}

bool LASReader::validateHeader(const LASHeader& header)
{
    if (strncmp(header.signature, "LASF", 4) != 0)
    {
        m_lastError = "Invalid LAS file signature";
        return false;
    }
    
    if (header.versionMajor != 1)
    {
        m_lastError = "Unsupported LAS major version: " + 
                     std::to_string(header.versionMajor);
        return false;
    }
    
    if (header.versionMinor < 0 || header.versionMinor > 4)
    {
        m_lastError = "Unsupported LAS minor version: " + 
                     std::to_string(header.versionMinor);
        return false;
    }
    
    if (header.pointDataRecordFormat > 10)
    {
        m_lastError = "Unsupported point data record format: " + 
                     std::to_string(header.pointDataRecordFormat);
        return false;
    }
    
    return true;
}

template<typename PointType>
bool LASReader::readPoints(std::ifstream& file, PointCloudData& pointCloud, 
                           const LASHeader& header, size_t points)
{
    PointType point;
    
    for (size_t i = 0; i < points; ++i)
    {
        if (!file.read(reinterpret_cast<char*>(&point), sizeof(PointType)))
        {
            break;
        }
        
        PointXYZRGBI pcdPoint;
        
        pcdPoint.xyz.x() = static_cast<float>(point.x * header.xScaleFactor + header.xOffset);
        pcdPoint.xyz.y() = static_cast<float>(point.y * header.yScaleFactor + header.yOffset);
        pcdPoint.xyz.z() = static_cast<float>(point.z * header.zScaleFactor + header.zOffset);
        
        pcdPoint.intensity = static_cast<float>(point.intensity);
        
        if constexpr (std::is_same<PointType, LASPointFormat2>::value || 
                      std::is_same<PointType, LASPointFormat3>::value)
        {
            pcdPoint.rgb[0] = static_cast<float>(point.red) / 65535.0f;
            pcdPoint.rgb[1] = static_cast<float>(point.green) / 65535.0f;
            pcdPoint.rgb[2] = static_cast<float>(point.blue) / 65535.0f;
        }
        
        pointCloud.addPoint(pcdPoint);
    }
    
    return true;
}

template bool LASReader::readPoints<LASPointFormat0>(std::ifstream&, PointCloudData&, 
                                                       const LASHeader&, size_t);
template bool LASReader::readPoints<LASPointFormat1>(std::ifstream&, PointCloudData&, 
                                                       const LASHeader&, size_t);
template bool LASReader::readPoints<LASPointFormat2>(std::ifstream&, PointCloudData&, 
                                                       const LASHeader&, size_t);
template bool LASReader::readPoints<LASPointFormat3>(std::ifstream&, PointCloudData&, 
                                                       const LASHeader&, size_t);
