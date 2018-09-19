/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_LOCALIZATION_MSF_COMMON_FILE_UTILITY_H_
#define MODULES_LOCALIZATION_MSF_COMMON_FILE_UTILITY_H_

#include <stdio.h>  
#include <node/openssl/md5.h>  
#include <string>  

namespace apollo {
namespace localization {
namespace msf {

const size_t UCHAR_MD5LENTH = 16;
const size_t CHAR_MD5LENTH = 33;

/**@brief Compute file md5 given a file path. */
void ComputeFileMd5(const std::string &file_path,
                      unsigned char res[UCHAR_MD5LENTH]);
void ComputeFileMd5(const std::string &file_path,
                      char res[CHAR_MD5LENTH]);
/**@brief Compute file md5 given a binary chunk. */
void ComputeBinaryMd5(const unsigned char* binary,
                        const unsigned int &size,
                        unsigned char res[UCHAR_MD5LENTH]);
void ComputeBinaryMd5(const unsigned char* binary,
                        const unsigned int &size,
                        char res[CHAR_MD5LENTH]);

}  // msf
}  // localization
}  // apollo

#endif // MODULES_LOCALIZATION_MSF_COMMON_FILE_UTILITY_H_
