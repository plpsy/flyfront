/*********************************************************
 * Copyright (C ) 2020 Flyslice Technologies Co.,Ltd
 * Author : Licheng Gu
 * **********************************************************/
#include <sys/fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h> 
#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <inttypes.h>

#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include "flyslice_cnn_api.h"

#define FLYSLICE_DEV_FILE	"/dev/flyslice_cdev0"
#define FPGA_690T_NUMBER				4

#define FPGA_690T_BIN_DDR_OFFSET		0
#define FPGA_690T_CFG_DDR_OFFSET		0
#define FPGA_690T_IMG_DDR_OFFSET		0

#define USER_BAR_PHY_ADDR				0x48300000//0xf7900000
#define USER_BAR_PHY_LEN				(1024*1024)	//1MB
#define BYPASS_BAR_PHY_ADDR				0x48500000//0xf7800000
#define BYPASS_BAR_PHY_LEN				(1024*1024)	//1MB
#define YOLO_ONE_OBJ_SIZE				16

#define FPGA_690T_REG(b, i, r)			((b) + (0x20*(i)) + (r) + 0x100)
#define FPGA_690T_START_REG(b, i)		((b) + (8*((i) - 1)) + 0x100)
#define FPGA_690T_FINISH_REG(b, i)		((b) + (8*((i) - 1)) + 0x104)

#define barrier() __asm__ __volatile__("": : :"memory")
/* Use 'x' as magic number */
#define FLY_IOC_MAGIC	'f'

/* Fly F->46(ASCII), L->6C(ASCII), y->79(ASCII) C->C; */
#define FLY_XCL_MAGIC 0X466C79
enum FLY_IOC_TYPES {
	FLY_IOC_NOP,
	FLY_IOC_INFO,
	FLY_IOC_DUMP,
	FLY_IOC_IMPORT,
	FLY_IOC_CNN_RES_LAYERS,
	FLY_IOC_MAX
};
struct __attribute__((__packed__)) fly_ioc_base {
	unsigned int magic;
	unsigned int command;
};
struct __attribute__((__packed__))  fly_ioc_dump
{
	struct fly_ioc_base base;
	unsigned long long dump_addr;
	unsigned long long dev_addr;
	unsigned long size;
};
struct fly_ioc_import {
	struct fly_ioc_base base;
	unsigned long long data_addr;
	unsigned long long dev_addr;
	unsigned long size;
};

#define FLY_IOCIMPORT 	_IOWR(FLY_IOC_MAGIC, FLY_IOC_IMPORT,	struct fly_ioc_import)
#define FLY_IOCDUMP		_IOWR(FLY_IOC_MAGIC, FLY_IOC_DUMP,	struct fly_ioc_dump)

#define PAGE_SIZE	4096UL
#define MAP_MASK	(PAGE_SIZE - 1)
static void *_mmap4bar(off_t phy_addr, int size)
{	
	int fd;
	void *map_base, *virt_addr;	

	if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
		return NULL;
	}

    printf("[fly_mmap4bar]>>>>> Map %d bytes on Phy address 0x%llx\n", size, phy_addr);
    
    /* Map*/
    map_base = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phy_addr & ~MAP_MASK);
    if (map_base == (void *) -1) {
		return NULL;
	}
    printf("BAR Memory mapped at address %p.\n", map_base);
    
    virt_addr = map_base + (phy_addr & MAP_MASK);
	close(fd);
	return virt_addr;
}

static inline uint32_t fly_reg_rd(volatile uint32_t *reg) 
{
	usleep(1000);
	return *reg;
}	

static inline void fly_reg_wr(uint32_t val, volatile uint32_t *reg) 
{
	usleep(1000);
	reg[0] = val;
}

static int fly_ioctl(int fd, unsigned int cmd, const unsigned char *data) 
{
	if (fd < 0)
        return -1;
    return ioctl(fd, cmd, data);
}

static int _fly_dump(const char* dev, unsigned long long offset, 
						const unsigned char *data, int len, int id) 
{
	int fd, rc;
	struct fly_ioc_dump ioc;
	
	if ((fd = open(dev, O_WRONLY)) <= 0) {
		perror("Could not open device file.");
		return -1;
	}
	memset(&ioc, 0, sizeof(ioc));
	ioc.base.magic = FLY_XCL_MAGIC;
	ioc.dump_addr = (unsigned long long)data;
	ioc.size = (unsigned long)len;
	ioc.dev_addr = offset;
	
	rc = fly_ioctl(fd, FLY_IOCDUMP, (unsigned char *)&ioc);
	close(fd);
	return rc;
}

static int _fly_import(const char* dev, unsigned long long offset, 
						const unsigned char *data, int len) 
{
	int fd, rc;
	struct fly_ioc_import ioc;
	
	if ((fd = open(dev, O_WRONLY)) <= 0) {
		perror("Could not open device file.");
		return -1;
	}
	memset(&ioc, 0, sizeof(ioc));
	ioc.base.magic = FLY_XCL_MAGIC;
	ioc.data_addr = (unsigned long long)data;
	ioc.size = (unsigned long)len;
	ioc.dev_addr = offset;
	
	rc = fly_ioctl(fd, FLY_IOCIMPORT, (unsigned char *)&ioc);
	close(fd);
	return rc;
}

static int fpga_trigger(volatile uint32_t *start, volatile uint32_t *finish, uint32_t timeout)
{
	int need_timeout = timeout;
	uint32_t done;
	struct timespec ts_start, ts_end;
	
	if (!start || !finish)
		return -1;
	clock_gettime(CLOCK_MONOTONIC, &ts_start);
	fly_reg_wr(0, finish);
	fly_reg_wr(1, start);
	barrier();
	if (!fly_reg_rd(start))
		return -1;
	do {
		done = fly_reg_rd(finish);
		if (done) {
			clock_gettime(CLOCK_MONOTONIC, &ts_end);
			/* subtract the start time from the end time */
			// timespec_sub(&ts_end, &ts_start);
			// printf("Spend %ld.%09ld seconds (total).\n", ts_end.tv_sec, ts_end.tv_nsec);
			return done;
		}
		if (need_timeout) {
			timeout--;
			if (!timeout)
				break;
		}
	} while(1);
	return -1;
}

FLYCODE loading_690t(int id, const uint8_t *data, int len)
{
	FLYCODE rc;
	volatile void *userbar_addr = NULL;
	
	if (id > FPGA_690T_NUMBER || id < 1 || len <= 0 || !data) {
		printf("ERROR: INVALID data or params!\n");
		return -FLYCODE_INVALID_DATA;
	}
	
	rc = _fly_import(FLYSLICE_DEV_FILE, FPGA_690T_BIN_DDR_OFFSET, data, len);
	if (rc < 0) {
		printf("ERROR: downloading FPGA bin file error (rc=%d).\n", rc);
		return -FLYCODE_LOADING_ERROR;
	}
	
	userbar_addr = (unsigned int *)_mmap4bar(USER_BAR_PHY_ADDR, USER_BAR_PHY_LEN);
	if (!userbar_addr) {
		printf("ERROR: mapping FPGA USER BAR space failed.\n");
		return -FLYCODE_MMAPING_FAILED;
	}
	fly_reg_wr(FPGA_690T_BIN_DDR_OFFSET, userbar_addr + 0x108);
	barrier();
	fly_reg_wr(len, userbar_addr + 0x10C);
	barrier();
	fly_reg_wr(id, userbar_addr + 0x110);
	barrier();
	munmap((void*)userbar_addr, USER_BAR_PHY_LEN);
	return FLYCODE_OK;
}

FLYCODE running_690t(int id)
{
	FLYCODE rc = FLYCODE_OK;
	volatile void *userbar_addr = NULL;
	
	if (id > FPGA_690T_NUMBER || id < 1) {
		printf("ERROR: INVALID data or params!\n");
		return -FLYCODE_INVALID_DATA;
	}
	userbar_addr = (unsigned int *)_mmap4bar(USER_BAR_PHY_ADDR, USER_BAR_PHY_LEN);
	if (!userbar_addr) {
		printf("ERROR: mapping FPGA USER BAR space failed.\n");
		return -FLYCODE_MMAPING_FAILED;
	}
	
	if (fpga_trigger((uint32_t *)(userbar_addr + 0x100), (uint32_t *)(userbar_addr + 0x104), 0) < 0)
		rc = -FLYCODE_START_FPGA_FAILED;
	munmap((void*)userbar_addr, USER_BAR_PHY_LEN);
	return rc;
}

FLYCODE loading_cnn_cfg(int id, const uint8_t *data, int len)
{
	FLYCODE rc = FLYCODE_OK;
	volatile void *bypassbar_addr = NULL;
	
	if (id > FPGA_690T_NUMBER || id < 1 || len <= 0 || !data) {
		printf("ERROR: INVALID data or params!\n");
		return -FLYCODE_INVALID_DATA;
	}

	rc = _fly_import(FLYSLICE_DEV_FILE, FPGA_690T_CFG_DDR_OFFSET, data, len);
	if (rc < 0) {
		printf("ERROR: downloading CNN configuration file error (rc=%d).\n", rc);
		return -FLYCODE_LOADING_ERROR;
	}

	bypassbar_addr = (unsigned int *)_mmap4bar(BYPASS_BAR_PHY_ADDR, BYPASS_BAR_PHY_LEN);
	if (!bypassbar_addr) {
		printf("ERROR: mapping FPGA Bypass BAR space failed.\n");
		return -FLYCODE_MMAPING_FAILED;
	}
	fly_reg_wr(FPGA_690T_CFG_DDR_OFFSET, FPGA_690T_REG(bypassbar_addr, id, 0x04));
	barrier();
	
	fly_reg_wr(0x8000000, FPGA_690T_REG(bypassbar_addr, id, 0x0C));
	barrier();
	
	fly_reg_wr(1, FPGA_690T_REG(bypassbar_addr, id, 0x10)); // set cfg loading mode
	barrier();

	fly_reg_wr(0x10000000, FPGA_690T_REG(bypassbar_addr, id, 0x14)); // set cfg loading address
	barrier();

	if (fpga_trigger((uint32_t *)FPGA_690T_START_REG(bypassbar_addr, id), 
						(uint32_t *)FPGA_690T_FINISH_REG(bypassbar_addr, id), 0) < 0)
		rc = -FLYCODE_START_FPGA_FAILED;
	else
		rc = FLYCODE_OK;
	munmap((void*)bypassbar_addr, BYPASS_BAR_PHY_LEN);
	return rc;
}

FLYCODE fpga_inference(int id, const uint8_t *img_data, int img_len, uint8_t *result, int *objs)
{
	FLYCODE rc;
	uint32_t obj_num, size, dect = 0;
	volatile void *bypassbar_addr = NULL;
	
	if (id > FPGA_690T_NUMBER || id < 1 || img_len <= 0 || !objs || !result || !img_data) {
		printf("ERROR: INVALID data or params!\n");
		return -FLYCODE_INVALID_DATA;
	}
	rc = _fly_import(FLYSLICE_DEV_FILE, FPGA_690T_IMG_DDR_OFFSET, img_data, img_len);
	if (rc < 0) {
		printf("ERROR: downloading image data error (rc=%d).\n", rc);
		return -FLYCODE_LOADING_ERROR;
	}
	bypassbar_addr = (unsigned int *)_mmap4bar(BYPASS_BAR_PHY_ADDR, BYPASS_BAR_PHY_LEN);
	if (!bypassbar_addr) {
		printf("ERROR: mapping FPGA Bypass BAR space failed.\n");
		return -FLYCODE_MMAPING_FAILED;
	}
	fly_reg_wr(FPGA_690T_IMG_DDR_OFFSET, FPGA_690T_REG(bypassbar_addr, id, 0x00));
	barrier();
	fly_reg_wr(img_len, FPGA_690T_REG(bypassbar_addr, id, 0x08));
	barrier();
	fly_reg_wr(2, FPGA_690T_REG(bypassbar_addr, id, 0x10)); // set image loading mode
	barrier();
	fly_reg_wr(0x10000000, FPGA_690T_REG(bypassbar_addr, id, 0x14)); // set image loading address
	barrier();
	if (fpga_trigger((uint32_t *)FPGA_690T_START_REG(bypassbar_addr, id), (uint32_t *)FPGA_690T_FINISH_REG(bypassbar_addr, id), 0) < 0) {
		rc = -FLYCODE_START_FPGA_FAILED;
		goto INF_OUT;
	}

	do {
		dect = fly_reg_rd(FPGA_690T_REG(bypassbar_addr, id, 0x18));	
	} while (!dect);
	obj_num = fly_reg_rd(FPGA_690T_REG(bypassbar_addr, id, 0x1C));
	*objs = obj_num;
	size = YOLO_ONE_OBJ_SIZE * obj_num;
	rc = _fly_dump(FLYSLICE_DEV_FILE, 0x10000000, result, size, 0);
	if (rc < 0) {
		printf("ERROR: get inference result error(rc=%d).\n", rc);
		rc = -FLYCODE_DMA_ERROR;
	}
	rc = FLYCODE_OK;
INF_OUT:
	munmap((void*)bypassbar_addr, BYPASS_BAR_PHY_LEN);
	return rc;
}

