/*
 * fs.c
 *
 *  Created on: 06.01.2019
 *      Author: Erich Styger
 */
#include "fs.h"
#include "errors.h"
#include "src/fs/W25Q128/W25Q128.h"
#include "src/fs/littleFS/lfs.h"
#include "src/UTIL1/UTIL1.h"
#include "src/printf/printf.h"

#define FS_FILE_NAME_SIZE  32 /* Length of file name, used in buffers */

/* variables used by the file system */
static bool FS_isMounted = false;
static lfs_t FS_lfs;

static int block_device_read(const struct lfs_config *c, lfs_block_t block,  lfs_off_t off, void *buffer, lfs_size_t size) {
  uint8_t res;

  res = W25_Read(block * c->block_size + off, buffer, size);
  if (res != ERR_OK) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

int block_device_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
  uint8_t res;

  res = W25_ProgramPage(block * c->block_size + off, buffer, size);
  if (res != ERR_OK) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

int block_device_erase(const struct lfs_config *c, lfs_block_t block) {
  uint8_t res;

  res = W25_EraseSector4K(block * c->block_size);
  if (res != ERR_OK) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

int block_device_sync(const struct lfs_config *c) {
  return LFS_ERR_OK;
}

// configuration of the file system is provided by this struct
const struct lfs_config FS_cfg = {
  // block device operations
  .read  = block_device_read,
  .prog  = block_device_prog,
  .erase = block_device_erase,
  .sync  = block_device_sync,

  .read_size = 256,
  .prog_size = 256,
  .block_size = 4096,
  .block_count = 4096,
  .block_cycles = 500,
  .cache_size = 256,
  .lookahead_size = 512,

};

uint8_t FS_WriteToFile(const char *filePath, uint8_t *buf, size_t bufSize) {
  lfs_file_t file;

  if (!FS_isMounted) {
    return ERR_FAILED;
  }
  /* Delete existing file */
  FS_RemoveFile(filePath);

  /* Open file for writing, create if necessary */
  if (lfs_file_open(&FS_lfs, &file, filePath, LFS_O_WRONLY | LFS_O_CREAT) < 0) {
    return ERR_FAILED;
  }

  /* Write supplied buffer to opened file */
  if (lfs_file_write(&FS_lfs, &file, buf, bufSize) < 0) {
    lfs_file_close(&FS_lfs, &file);
    return ERR_FAILED;
  }
  
  /* Clean up and exit */
  lfs_file_close(&FS_lfs, &file);
  return ERR_FAILED;
}

#if 0

// uint8_t FS_RunBenchmark(CLS1_ConstStdIOType *io) {
uint8_t FS_SaveConfig(void) {
  lfs_file_t file;
  int result;
  uint32_t i;
  uint8_t read_buf[10];

  uint32_t time, startTime;
  int32_t start_mseconds, mseconds;

  if (!FS_isMounted) {
    printf_("ERROR: File system is not mounted.\r\n");
    return ERR_FAILED;
  }
  /* write conf file */
  printf_("Save conf file:\r\n");
  printf_("Delete existing conf.txt file...\r\n");
  FS_RemoveFile("./config.txt");

  printf_("Create conf.txt file...\r\n");
  startTime = millis();
  if (lfs_file_open(&FS_lfs, &file, "./config.txt", LFS_O_WRONLY | LFS_O_CREAT) < 0) {
    printf_("*** Failed creating conf.txt file!\r\n");
    return ERR_FAILED;
  }
  for (i = 0; i < 1024; i++) {
    if (lfs_file_write(&FS_lfs, &file, "configure ", sizeof("configure ") - 1) < 0) {
      printf_("*** Failed writing conf file!\r\n");
      lfs_file_close(&FS_lfs, &file);
      return ERR_FAILED;
    }
  }
  lfs_file_close(&FS_lfs, &file);
  mseconds = millis() - startTime;
  printf_("%u", mseconds);
  printf_(" ms for writing (");
  printf_("%u", (100 * 1000) / mseconds);
  printf_(" kB/s)\r\n");

  /* read benchmark */
  printf_("Read conf file...\r\n");
  startTime = millis();
  if (lfs_file_open(&FS_lfs, &file, "./config.txt", LFS_O_RDONLY) < 0) {
    printf_("*** Failed opening conf.txt file!\r\n");
    return ERR_FAILED;
  }
  for (i = 0; i < 10240; i++) {
    if (lfs_file_read(&FS_lfs, &file, &read_buf[0], sizeof(read_buf)) < 0) {
      printf_("*** Failed reading conf file!\r\n");
      lfs_file_close(&FS_lfs, &file);
      return ERR_FAILED;
    }
  }
  lfs_file_close(&FS_lfs, &file);
  mseconds = millis() - startTime;
  printf_("%u", mseconds);
  printf_(" ms for reading (");
  printf_("%u", (100 * 1000) / mseconds);
  printf_(" kB/s)\r\n");

  printf_("done!\r\n");
  return ERR_OK;
}

#endif

uint8_t FS_ReadFromFile(const char *srcPath, uint8_t *buf, size_t bufSize) {
  lfs_file_t fsrc;
  int result;

  /* open source file */
  result = lfs_file_open(&FS_lfs, &fsrc, srcPath, LFS_O_RDONLY);
  if (result < 0) {
    return ERR_FAILED;
  }

  result = lfs_file_read(&FS_lfs, &fsrc, buf, bufSize);
  if (result != bufSize) {
    return ERR_FAILED;
  }

  /* close source file */
  result = lfs_file_close(&FS_lfs, &fsrc);
  if (result < 0) {
    return ERR_FAILED;
  }
  return ERR_OK;
}

uint8_t FS_Format(void) {
  int res;

  if (FS_isMounted) {
    printf_("File system is already mounted, unmount it first.\r\n");
    return ERR_FAILED;
  }
  printf_("Formatting ...");
  res = lfs_format(&FS_lfs, &FS_cfg);
  if (res == LFS_ERR_OK) {
    printf_(" done.\r\n");
    return ERR_OK;
  } else {
    printf_(" FAILED!\r\n");
    return ERR_FAILED;
  }
}

uint8_t FS_Mount(void) {
  int res;

  if (FS_isMounted) {
    printf_("File system is already mounted, unmount it first.\r\n");
    return ERR_FAILED;
  }
  printf_("Mounting ...");
  res = lfs_mount(&FS_lfs, &FS_cfg);
  if (res == LFS_ERR_OK) {
    printf_(" done.\r\n");
    FS_isMounted = true;
    return ERR_OK;
  } else {
    printf_(" FAILED!\r\n");
    return ERR_FAILED;
  }
}

uint8_t FS_Unmount(void) {
  int res;

  if (!FS_isMounted) {
    printf_("File system is already unmounted.\r\n");
    return ERR_FAILED;
  }
  printf_("Unmounting ...");
  res = lfs_unmount(&FS_lfs);
  if (res == LFS_ERR_OK) {
    printf_(" done.\r\n");
    FS_isMounted = false;
    return ERR_OK;
  } else {
    printf_(" FAILED!\r\n");
    return ERR_FAILED;
  }
}

// uint8_t FS_Dir(const char *path, CLS1_ConstStdIOType *io) {
uint8_t FS_Dir(const char *path) {
  int res;
  lfs_dir_t dir;
  struct lfs_info info;

  if (!FS_isMounted) {
    printf_("File system is not mounted, mount it first.\r\n");
    return ERR_FAILED;
  }
  if (path == NULL) {
    path = "/"; /* default path */
  }
  res = lfs_dir_open(&FS_lfs, &dir, path);
  if (res != LFS_ERR_OK) {
    printf_("FAILED lfs_dir_open()!\r\n");
    return ERR_FAILED;
  }
  for (;;) {
    res = lfs_dir_read(&FS_lfs, &dir, &info);
    if (res < 0) {
      printf_("FAILED lfs_dir_read()!\r\n");
      return ERR_FAILED;
    }
    if (res == 0) { /* no more files */
      break;
    }
    switch (info.type) {
    case LFS_TYPE_REG: printf_("reg "); break;
    case LFS_TYPE_DIR: printf_("dir "); break;
    default:           printf_("?   "); break;
    }
    static const char *prefixes[] = {"", "K", "M", "G"}; /* prefixes for kilo, mega and giga */
    for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--) {
      if (info.size >= (1 << 10 * i) - 1) {
        printf_("%*u%sB ", 4 - (i != 0), info.size >> 10 * i, prefixes[i]);
        break;
      }
    } /* for */
    printf_(info.name);
    printf_("\r\n");
  } /* for */
  res = lfs_dir_close(&FS_lfs, &dir);
  if (res != LFS_ERR_OK) {
    printf_("FAILED lfs_dir_close()!\r\n");
    return ERR_FAILED;
  }
  return ERR_OK;
}

bool FS_Exists(const char* path) {
  struct lfs_info info;

  if (!FS_isMounted) {
    printf_("File system is not mounted, mount it first.\r\n");
    return ERR_FAILED;
  }
  if (path == NULL) {
    printf_("Path not specified.\r\n");
    return ERR_FAILED;
  }
  int rc = lfs_stat(&FS_lfs, path, &info);
  return rc == 0;
}

uint8_t FS_CreateFile(const char *dstPath, int flags) {
  lfs_file_t fdst;
  uint8_t res =  ERR_OK;
  int result;

  if (!FS_isMounted) {
    printf_("ERROR: File system is not mounted.\r\n");
    return ERR_FAILED;
  }
  /* create destination file */
  result = lfs_file_open(&FS_lfs, &fdst, dstPath, flags);
  if (result < 0) {
    printf_("*** Failed opening file to be created!\r\n");
    return ERR_FAILED;
  }
  /* close destination file */
  result = lfs_file_close(&FS_lfs, &fdst);
  if (result < 0) {
    printf_("*** Failed closing created file!\r\n");
    res = ERR_FAILED;
  }
  return ERR_OK;
}

// uint8_t FS_CopyFile(const char *srcPath, const char *dstPath, CLS1_ConstStdIOType *io) {
uint8_t FS_CopyFile(const char *srcPath, const char *dstPath) {
  lfs_file_t fsrc, fdst;
  uint8_t res =  ERR_OK;
  int result, nofBytesRead;
  uint8_t buffer[32];   /* copy buffer */

  if (!FS_isMounted) {
    printf_("ERROR: File system is not mounted.\r\n");
    return ERR_FAILED;
  }

  /* open source file */
  result = lfs_file_open(&FS_lfs, &fsrc, srcPath, LFS_O_RDONLY);
  if (result < 0) {
    printf_("*** Failed opening source file!\r\n");
    return ERR_FAILED;
  }
  /* create destination file */
  result = lfs_file_open(&FS_lfs, &fdst, dstPath, LFS_O_WRONLY | LFS_O_CREAT);
  if (result < 0) {
    lfs_file_close(&FS_lfs, &fsrc);
    printf_("*** Failed opening destination file!\r\n");
    return ERR_FAILED;
  }
  /* now copy source to destination */
  for (;;) {
    nofBytesRead = lfs_file_read(&FS_lfs, &fsrc, buffer, sizeof(buffer));
    if (nofBytesRead < 0) {
      printf_("*** Failed reading source file!\r\n");
      res = ERR_FAILED;
      break;
    }
    if (nofBytesRead == 0) { /* end of file */
      break;
    }
    result = lfs_file_write(&FS_lfs, &fdst, buffer, nofBytesRead);
    if (result < 0) {
      printf_("*** Failed writing destination file!\r\n");
      res = ERR_FAILED;
      break;
    }
  } /* for */
  /* close all files */
  result = lfs_file_close(&FS_lfs, &fsrc);
  if (result < 0) {
    printf_("*** Failed closing source file!\r\n");
    res = ERR_FAILED;
  }
  result = lfs_file_close(&FS_lfs, &fdst);
  if (result < 0) {
    printf_("*** Failed closing destination file!\r\n");
    res = ERR_FAILED;
  }
  return ERR_OK;
}

// uint8_t FS_MoveFile(const char *srcPath, const char *dstPath, CLS1_ConstStdIOType *io) {
uint8_t FS_MoveFile(const char *srcPath, const char *dstPath) {
  if (!FS_isMounted) {
    printf_("ERROR: File system is not mounted.\r\n");
    return ERR_FAILED;
  }
  if (lfs_rename(&FS_lfs, srcPath, dstPath) < 0) {
    printf_("ERROR: failed renaming file or directory.\r\n");
    return ERR_FAILED;
  }
  return ERR_OK;
}

static uint8_t readFromFile(void *hndl, uint32_t addr, uint8_t *buf, size_t bufSize) {
  lfs_file_t *fp;

  fp = (lfs_file_t*)hndl;
  if (lfs_file_read(&FS_lfs, fp, buf, bufSize) < 0) {
    return ERR_FAILED;
  }
  return ERR_OK;
}

// uint8_t FS_RemoveFile(const char *filePath, CLS1_ConstStdIOType *io) {
uint8_t FS_RemoveFile(const char *filePath) {
  int result;

  if (!FS_isMounted) {
    printf_("ERROR: File system is not mounted.\r\n");
    return ERR_FAILED;
  }
  result = lfs_remove(&FS_lfs, filePath);
  if (result < 0) {
    printf_("ERROR: Failed removing file.\r\n");
    return ERR_FAILED;
  }
  return ERR_OK;
}

// uint8_t FS_RunBenchmark(CLS1_ConstStdIOType *io) {
uint8_t FS_RunBenchmark(void) {
  lfs_file_t file;
  int result;
  uint32_t i;
  uint8_t read_buf[10];
  // TIMEREC time, startTime;
  uint32_t time, startTime;
  int32_t start_mseconds, mseconds;

  if (!FS_isMounted) {
    printf_("ERROR: File system is not mounted.\r\n");
    return ERR_FAILED;
  }
  /* write benchmark */
  printf_("Benchmark: write/copy/read a 100kB file:\r\n");
  printf_("Delete existing benchmark files...\r\n");
  FS_RemoveFile("./bench.txt");
  FS_RemoveFile("./copy.txt");

  printf_("Create benchmark file...\r\n");
  startTime = millis();
  if (lfs_file_open(&FS_lfs, &file, "./bench.txt", LFS_O_WRONLY | LFS_O_CREAT) < 0) {
    printf_("*** Failed creating benchmark file!\r\n");
    return ERR_FAILED;
  }
  for (i = 0; i < 10240; i++) {
    if (lfs_file_write(&FS_lfs, &file, "benchmark ", sizeof("benchmark ") - 1) < 0) {
      printf_("*** Failed writing file!\r\n");
      lfs_file_close(&FS_lfs, &file);
      return ERR_FAILED;
    }
  }
  lfs_file_close(&FS_lfs, &file);
  mseconds = millis() - startTime;
  printf_("%u", mseconds);
  printf_(" ms for writing (");
  printf_("%u", (100 * 1000) / mseconds);
  printf_(" kB/s)\r\n");

  /* read benchmark */
  printf_("Read 100kB benchmark file...\r\n");
  startTime = millis();
  if (lfs_file_open(&FS_lfs, &file, "./bench.txt", LFS_O_RDONLY) < 0) {
    printf_("*** Failed opening benchmark file!\r\n");
    return ERR_FAILED;
  }
  for (i = 0; i < 10240; i++) {
    if (lfs_file_read(&FS_lfs, &file, &read_buf[0], sizeof(read_buf)) < 0) {
      printf_("*** Failed reading file!\r\n");
      lfs_file_close(&FS_lfs, &file);
      return ERR_FAILED;
    }
  }
  lfs_file_close(&FS_lfs, &file);
  mseconds = millis() - startTime;
  printf_("%u", mseconds);
  printf_(" ms for reading (");
  printf_("%u", (100 * 1000) / mseconds);
  printf_(" kB/s)\r\n");

  /* copy benchmark */
  printf_("Copy 100kB file...\r\n");
  startTime = millis();
  // FS_CopyFile((const unsigned char*)"./bench.txt", (const unsigned char*)"./copy.txt");
  FS_CopyFile("./bench.txt", "./copy.txt");
  printf_("%u", mseconds);
  printf_(" ms for copy (");
  printf_("%u", (100 * 1000) / mseconds);
  printf_(" kB/s)\r\n");
  printf_("done!\r\n");
  return ERR_OK;
}

// static uint8_t FS_PrintStatus(CLS1_ConstStdIOType *io) {
uint8_t FS_PrintStatus(void) {
  uint8_t buf[24];

  printf_("FS%s", "\r\n");
  printf_("  mounted", FS_isMounted ? "yes\r\n" : "no\r\n");

  UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_count * FS_cfg.block_size);
  UTIL1_strcat(buf, sizeof(buf), " bytes\r\n");
  printf_("  space: %s", buf);

  UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.read_size);
  UTIL1_strcat(buf, sizeof(buf), "\r\n");
  printf_("  read_size: %s", buf);

  UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.prog_size);
  UTIL1_strcat(buf, sizeof(buf), "\r\n");
  printf_("  prog_size: %s", buf);

  UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_size);
  UTIL1_strcat(buf, sizeof(buf), "\r\n");
  printf_("  block_size: %s", buf);

  UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_count);
  UTIL1_strcat(buf, sizeof(buf), "\r\n");
  printf_("  block_count: %s", buf);

  UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.lookahead_size);
  UTIL1_strcat(buf, sizeof(buf), "\r\n");
  printf_("  lookahead_size: %s", buf);
  return ERR_OK;
}

uint8_t printMemory(void *hndl, uint32_t startAddr, uint32_t endAddr, uint8_t addrSize, uint8_t bytesPerLine, uint8_t (*readfp)(void *, uint32_t, uint8_t*, size_t))
{
  #define NOF_BYTES_PER_LINE 32 /* how many bytes are shown on a line. This defines as well the chunk size we read from memory */
  #define MAX_NOF_BYTES_PER_LINE 32
  uint8_t buf[MAX_NOF_BYTES_PER_LINE]; /* this is the chunk of data we get (per line in output) */
  uint8_t str[3*MAX_NOF_BYTES_PER_LINE+((MAX_NOF_BYTES_PER_LINE+1)/8)+1]; /* maximum string for output:
                                              - '3*' because each byte is 2 hex digits plus a space
                                              - '(NOF_BYTES_PER_LINE+1)/8' because we add a space between every 8 byte block
                                              - '+1' for the final zero byte */
  uint32_t addr;
  uint8_t res=0, j, bufSize;
  uint8_t ch;

  if (endAddr<startAddr) {
    printf_("\r\n*** End address must be larger or equal than start address\r\n");
    return ERR_RANGE;
  }
  for(addr=startAddr; addr<=endAddr; /* nothing */ ) {
    if (endAddr-addr+1 >= bytesPerLine) { /* read only part of buffer */
      bufSize = bytesPerLine; /* read full buffer */
    } else {
      bufSize = (uint8_t)(endAddr-addr+1);
    }
    if (readfp(hndl, addr, buf, bufSize)!=ERR_OK) {
      printf_("\r\n*** Read failed!\r\n");
      return ERR_FAILED;
    }
    if (res != ERR_OK) {
      printf_("\r\n*** Failure reading memory block!\r\n");
      return ERR_FAULT;
    }
    /* write address */
    UTIL1_strcpy(str, sizeof(str), (unsigned char*)"0x");
    UTIL1_strcatNumHex(str, sizeof(str), addr, addrSize);
    UTIL1_chcat(str, sizeof(str), ':');
    printf_(str);
    /* write data in hex */
    str[0] = '\0';
    for (j=0; j<bufSize; j++) {
      if ((j)==0) {
        UTIL1_chcat(str, sizeof(str), ' ');
      }
      UTIL1_strcatNum8Hex(str, sizeof(str), buf[j]);
      UTIL1_chcat(str, sizeof(str), ' ');
    }
    for (/*empty*/; j<bytesPerLine; j++) { /* fill up line */
      UTIL1_strcat(str, sizeof(str), (unsigned char*)"-- ");
    }
    printf_(str);
    /* write in ASCII */
    printf_(" ");
    for (j=0; j<bufSize; j++) {
      ch = buf[j];
      if (ch >= ' ' && ch <= 0x7f) {
        printf_("%c", ch);
      } else {
        printf_(".");
      }
    }
    for (/*empty*/; j<bytesPerLine; j++) { /* fill up line */
      UTIL1_strcat(str, sizeof(str), (unsigned char*)"-- ");
    }
    printf_("\r\n");
    addr += bytesPerLine;
  }
  return ERR_OK;
}

// uint8_t FS_PrintHexFile(const char *filePath, CLS1_ConstStdIOType *io) {
uint8_t FS_PrintHexFile(const char *filePath) {
  lfs_file_t file;
  uint8_t res = ERR_OK;
  int32_t fileSize;
  int result;

  if (!FS_isMounted) {
    printf_("ERROR: File system is not mounted.\r\n");
    return ERR_FAILED;
  }
  result = lfs_file_open(&FS_lfs, &file, filePath, LFS_O_RDONLY);
  if (result < 0) {
    printf_("ERROR: Failed opening file.\r\n");
    return ERR_FAILED;
  }
  fileSize = lfs_file_size(&FS_lfs, &file);
  if (fileSize < 0) {
    printf_("ERROR: getting file size\r\n");
    lfs_file_close(&FS_lfs, &file);
    return ERR_FAILED;
  }
  res = printMemory(&file, 0, fileSize - 1, 4, 16, readFromFile);
  if (res != ERR_OK) {
    printf_("ERROR while calling PrintMemory()\r\n");
  }
  lfs_file_close(&FS_lfs, &file);
  return res;
}