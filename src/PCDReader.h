#ifndef PCDREADER_H
#define PCDREADER_H

#include "PointCloudData.h"
#include <string>
#include <fstream>

class PCDReader
{
public:
    PCDReader();
    ~PCDReader();
    
    bool read(const std::string& filename, PointCloudData& pointCloud);
    
    std::string getLastError() const { return m_lastError; }

private:
    bool readASCII(std::ifstream& file, PointCloudData& pointCloud, 
                   const std::vector<std::string>& fields, size_t points);
    bool readBinary(std::ifstream& file, PointCloudData& pointCloud, 
                    const std::vector<std::string>& fields, size_t points);
    
    std::string m_lastError;
};

#endif // PCDREADER_H
