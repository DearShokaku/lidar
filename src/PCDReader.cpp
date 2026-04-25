#include "PCDReader.h"
#include <sstream>
#include <iostream>
#include <limits>

PCDReader::PCDReader()
{
}

PCDReader::~PCDReader()
{
}

bool PCDReader::read(const std::string& filename, PointCloudData& pointCloud)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        m_lastError = "Cannot open file: " + filename;
        return false;
    }
    
    pointCloud.clear();
    
    std::string line;
    std::string dataType = "ascii";
    std::vector<std::string> fields;
    size_t points = 0;
    size_t width = 0;
    size_t height = 0;
    
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;
        
        if (keyword == "FIELDS")
        {
            std::string field;
            while (iss >> field)
            {
                fields.push_back(field);
            }
        }
        else if (keyword == "POINTS")
        {
            iss >> points;
        }
        else if (keyword == "WIDTH")
        {
            iss >> width;
        }
        else if (keyword == "HEIGHT")
        {
            iss >> height;
        }
        else if (keyword == "DATA")
        {
            iss >> dataType;
            break;
        }
    }
    
    if (points == 0 && width * height > 0)
    {
        points = width * height;
    }
    
    if (fields.empty())
    {
        m_lastError = "No fields defined in PCD file";
        return false;
    }
    
    if (dataType == "ascii")
    {
        return readASCII(file, pointCloud, fields, points);
    }
    else if (dataType == "binary" || dataType == "binary_compressed")
    {
        return readBinary(file, pointCloud, fields, points);
    }
    else
    {
        m_lastError = "Unknown data type: " + dataType;
        return false;
    }
}

bool PCDReader::readASCII(std::ifstream& file, PointCloudData& pointCloud, 
                          const std::vector<std::string>& fields, size_t points)
{
    int xIdx = -1, yIdx = -1, zIdx = -1;
    int intensityIdx = -1;
    int rIdx = -1, gIdx = -1, bIdx = -1;
    int rgbIdx = -1;
    
    for (size_t i = 0; i < fields.size(); ++i)
    {
        if (fields[i] == "x") xIdx = static_cast<int>(i);
        else if (fields[i] == "y") yIdx = static_cast<int>(i);
        else if (fields[i] == "z") zIdx = static_cast<int>(i);
        else if (fields[i] == "intensity") intensityIdx = static_cast<int>(i);
        else if (fields[i] == "r") rIdx = static_cast<int>(i);
        else if (fields[i] == "g") gIdx = static_cast<int>(i);
        else if (fields[i] == "b") bIdx = static_cast<int>(i);
        else if (fields[i] == "rgb") rgbIdx = static_cast<int>(i);
    }
    
    if (xIdx < 0 || yIdx < 0 || zIdx < 0)
    {
        m_lastError = "PCD file does not contain x, y, z fields";
        return false;
    }
    
    std::string line;
    size_t readPoints = 0;
    
    while (readPoints < points && std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        
        std::istringstream iss(line);
        std::vector<float> values;
        float val;
        
        while (iss >> val)
        {
            values.push_back(val);
        }
        
        if (values.size() < fields.size())
        {
            continue;
        }
        
        PointXYZRGBI point;
        point.xyz.x() = values[xIdx];
        point.xyz.y() = values[yIdx];
        point.xyz.z() = values[zIdx];
        
        if (intensityIdx >= 0 && intensityIdx < static_cast<int>(values.size()))
        {
            point.intensity = values[intensityIdx];
        }
        
        if (rIdx >= 0 && gIdx >= 0 && bIdx >= 0)
        {
            if (rIdx < static_cast<int>(values.size()))
                point.rgb[0] = values[rIdx] / 255.0f;
            if (gIdx < static_cast<int>(values.size()))
                point.rgb[1] = values[gIdx] / 255.0f;
            if (bIdx < static_cast<int>(values.size()))
                point.rgb[2] = values[bIdx] / 255.0f;
        }
        else if (rgbIdx >= 0 && rgbIdx < static_cast<int>(values.size()))
        {
            uint32_t rgb = static_cast<uint32_t>(values[rgbIdx]);
            point.rgb[0] = ((rgb >> 16) & 0xFF) / 255.0f;
            point.rgb[1] = ((rgb >> 8) & 0xFF) / 255.0f;
            point.rgb[2] = (rgb & 0xFF) / 255.0f;
        }
        
        pointCloud.addPoint(point);
        readPoints++;
    }
    
    return true;
}

bool PCDReader::readBinary(std::ifstream& file, PointCloudData& pointCloud, 
                           const std::vector<std::string>& fields, size_t points)
{
    int xIdx = -1, yIdx = -1, zIdx = -1;
    int intensityIdx = -1;
    int rgbIdx = -1;
    
    for (size_t i = 0; i < fields.size(); ++i)
    {
        if (fields[i] == "x") xIdx = static_cast<int>(i);
        else if (fields[i] == "y") yIdx = static_cast<int>(i);
        else if (fields[i] == "z") zIdx = static_cast<int>(i);
        else if (fields[i] == "intensity") intensityIdx = static_cast<int>(i);
        else if (fields[i] == "rgb") rgbIdx = static_cast<int>(i);
    }
    
    if (xIdx < 0 || yIdx < 0 || zIdx < 0)
    {
        m_lastError = "PCD file does not contain x, y, z fields";
        return false;
    }
    
    size_t pointSize = fields.size() * sizeof(float);
    std::vector<char> buffer(pointSize);
    
    for (size_t i = 0; i < points; ++i)
    {
        if (!file.read(buffer.data(), pointSize))
        {
            break;
        }
        
        float* values = reinterpret_cast<float*>(buffer.data());
        
        PointXYZRGBI point;
        point.xyz.x() = values[xIdx];
        point.xyz.y() = values[yIdx];
        point.xyz.z() = values[zIdx];
        
        if (intensityIdx >= 0)
        {
            point.intensity = values[intensityIdx];
        }
        
        if (rgbIdx >= 0)
        {
            uint32_t rgb = *reinterpret_cast<uint32_t*>(&values[rgbIdx]);
            point.rgb[0] = ((rgb >> 16) & 0xFF) / 255.0f;
            point.rgb[1] = ((rgb >> 8) & 0xFF) / 255.0f;
            point.rgb[2] = (rgb & 0xFF) / 255.0f;
        }
        
        pointCloud.addPoint(point);
    }
    
    return true;
}
