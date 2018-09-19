#include <string.h>
#include <vector>
#include <iostream>
#include "modules/localization/msf/common/util/file_utility.h"

namespace apollo {
namespace localization {
namespace msf {
const size_t BUFFER_SIZE = 20480000;

void ComputeFileMd5(const std::string &file_path,
                      unsigned char res[UCHAR_MD5LENTH]) {
    std::vector<unsigned char> buf(BUFFER_SIZE);
    unsigned char *buf_pt = &buf[0];

    FILE *file = fopen(file_path.c_str(), "rb");
    size_t total_size = 0;
    if (file) {
        int count = 1;
        while (!feof(file)){
            if (count > 1) {
                buf.resize(BUFFER_SIZE * count);
                buf_pt = &buf[count - 1];
            }

            size_t size = fread(buf_pt, sizeof(unsigned char), BUFFER_SIZE, file);
            total_size += size;
            
            ++count;
        }
    }
    else {
        std::cerr << "Can't find the file: " << file_path << std::endl;
        return;
    }

    ComputeBinaryMd5(&buf[0], total_size, res);
    fclose(file);
}

void ComputeFileMd5(const std::string &file_path,
                      char res[CHAR_MD5LENTH]) {
    std::vector<unsigned char> buf(BUFFER_SIZE);
    unsigned char *buf_pt = &buf[0];

    FILE *file = fopen(file_path.c_str(), "rb");
    size_t total_size = 0;
    if (file) {
        int count = 1;
        while (!feof(file)){
            if (count > 1) {
                buf.resize(BUFFER_SIZE * count);
                buf_pt = &buf[count - 1];
            }

            size_t size = fread(buf_pt, sizeof(unsigned char), BUFFER_SIZE, file);
            total_size += size;
            
            ++count;
        }
    }
    else {
        std::cerr << "Can't find the file: " << file_path << std::endl;
        return;
    }

    ComputeBinaryMd5(&buf[0], total_size, res);
    fclose(file);
}

void ComputeBinaryMd5(const unsigned char* binary,
                        const unsigned int &size,
                        unsigned char res[UCHAR_MD5LENTH]) {
    MD5_CTX ctx;  
    unsigned char md[16];  
      
    MD5_Init(&ctx);  
    MD5_Update(&ctx, binary, size);  
    MD5_Final(res, &ctx);  
}

void ComputeBinaryMd5(const unsigned char* binary,
                        const unsigned int &size,
                        char res[CHAR_MD5LENTH]) { 
    unsigned char md[UCHAR_MD5LENTH] = {"\0"};  
    char buf[CHAR_MD5LENTH] = {'\0'};  
    char tmp[3] = {'\0'};  

    ComputeBinaryMd5(binary, size, md);

    for (int i = 0; i < UCHAR_MD5LENTH; i++)  
    {  
        snprintf(tmp, sizeof(tmp), "%02X", md[i]);  
        strncat(buf, tmp, sizeof(tmp));  
    }  
  
    memcpy(res, buf, sizeof(buf));
}

}  // msf
}  // localization
}  // apollo