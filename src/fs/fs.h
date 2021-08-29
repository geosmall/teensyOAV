/*
 * fs.h
 *
 *  Created on: 06.01.2019
 *      Author: Erich Styger
 */

#ifndef SOURCES_FS_H_
#define SOURCES_FS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


#ifdef __cplusplus
extern "C" {
#endif

uint8_t FS_Init(void);

uint8_t FS_Format(void);

uint8_t FS_Mount(void);

uint8_t FS_Unmount(void);

uint8_t FS_Dir(const char *path);

bool FS_Exists(const char* path);

uint8_t FS_CreateFile(const char *dstPath, int flags);

uint8_t FS_RemoveFile(const char *filePath);

uint8_t FS_ReadFromFile(const char *srcPath, uint8_t *buf, size_t bufSize);

uint8_t FS_WriteToFile(const char *dstPath, uint8_t *buf, size_t bufSize);

uint8_t FS_PrintHexFile(const char *filePath);

uint8_t FS_RunBenchmark(void);

uint8_t FS_PrintStatus(void);

uint8_t FS_SaveConfig(void);


#ifdef __cplusplus
}
#endif

#endif /* SOURCES_FS_H_ */
