#define FLYSLICE_DEV_FILE	"/dev/flyslice_cdev0"

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
#include "cJSON.h"

#define FPGA_690T_BIN_DDR_OFFSET		0
#define FPGA_690T_CFG_DDR_OFFSET		0
#define FPGA_690T_IMG_DDR_OFFSET		0

#define USER_BAR_PHY_ADDR				0x48500000//0xf7900000
#define USER_BAR_PHY_LEN				(1024*1024)	//1MB
#define BYPASS_BAR_PHY_ADDR				0x48700000//0xf7800000
#define BYPASS_BAR_PHY_LEN				(1024*1024)	//1MB
#define YOLO_ONE_OBJ_SIZE				16

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

struct __attribute__((__packed__))  fly_yolo_obj
{
	uint16_t box_x0;
	uint16_t box_x1;
	uint16_t box_y0;
	uint16_t box_y1;
	float prob;
	uint16_t cls;
	uint8_t seq;
	uint8_t boxes;
};


static struct option const long_opts[] =
{
  {"config", required_argument, NULL, 'c'},
  {"file", required_argument, NULL, 'f'},
  {"id", required_argument, NULL, 'i'},
  {"output", required_argument, NULL, 'o'},
  {"fpga", required_argument, NULL, 't'},
  //{"address", required_argument, NULL, 'a'},
  //{"size", required_argument, NULL, 's'},
  //{"verbose", no_argument, NULL, 'v'},
  {"help", no_argument, NULL, 'h'},
  {0, 0, 0, 0}
};

static void *_mmap4bar(off_t phy_addr, int size);
static int fpga_690t_load(char *devicename, const char *filename, int id);
static int yolo_cfg_load(char *devicename, const char *filename, int id);
int yolo_test(char *devicename, const char *image_path, const char *out_path, int id);

/* Subtract timespec t2 from t1
 *
 * Both t1 and t2 must already be normalized
 * i.e. 0 <= nsec < 1000000000 */
static void timespec_sub(struct timespec *t1, const struct timespec *t2)
{
	assert(t1->tv_nsec >= 0);
	assert(t1->tv_nsec < 1000000000);
	assert(t2->tv_nsec >= 0);
	assert(t2->tv_nsec < 1000000000);
	t1->tv_sec -= t2->tv_sec;
	t1->tv_nsec -= t2->tv_nsec;
	if (t1->tv_nsec >= 1000000000) {
		t1->tv_sec++;
		t1->tv_nsec -= 1000000000;
	} else if (t1->tv_nsec < 0) {
		t1->tv_sec--;
		t1->tv_nsec += 1000000000;
	}
}

int msleep(long msec)
{
	struct timespec ts;
	int res;

	if (msec < 0) {
		return -1;
	}

	ts.tv_sec = msec / 1000;
	ts.tv_nsec = (msec % 1000) * 1000000;

	do {
		res = nanosleep(&ts, &ts);
	} while (res);

	return res;
}

#define SIZE_DEFAULT (4096)
#define FPGA_ID_DEFAULT (1)

static void usage(const char* name)
{
	int i = 0;
	fprintf(stderr, "%s\n\n", name);
	fprintf(stderr, "usage: %s [OPTIONS]\n\n", name);
	fprintf(stderr, "Test one image data on FPGA running YOLOVx\n\n");

	fprintf(stderr, "  -%c (--%s) yolo config data file for loading.\n", long_opts[i].val, long_opts[i].name); i++;
	fprintf(stderr, "  -%c (--%s) image data file for inference.\n", long_opts[i].val, long_opts[i].name); i++;
	fprintf(stderr, "  -%c (--%s) the number of 690t for starting, default is %d.\n", long_opts[i].val, long_opts[i].name, FPGA_ID_DEFAULT); i++; 
	fprintf(stderr, "  -%c (--%s) the result of inference ouput as bin.\n", long_opts[i].val, long_opts[i].name); i++; 
	fprintf(stderr, "  -%c (--%s) 690T FPGA BIT file for loading.\n", long_opts[i].val, long_opts[i].name); i++;
	//fprintf(stderr, "  -%c (--%s) address of the start address on the AXI bus\n", long_opts[i].val, long_opts[i].name); i++;
	//fprintf(stderr, "  -%c (--%s) size of a single transfer in bytes, default is %d.\n", long_opts[i].val, long_opts[i].name, SIZE_DEFAULT); i++;
	//fprintf(stderr, "  -%c (--%s) be more verbose during test\n", long_opts[i].val, long_opts[i].name); i++;
	fprintf(stderr, "  -%c (--%s) print usage help and exit\n", long_opts[i].val, long_opts[i].name); i++;
}

static uint32_t getopt_integer(char *optarg)
{
  int rc;
  uint32_t value;
  rc = sscanf(optarg, "0x%x", &value);
  if (rc <= 0)
    rc = sscanf(optarg, "%ul", &value);
  //fprintf(stderr, "sscanf() = %d, value = 0x%08x\n", rc, (unsigned int)value);
  return value;
}

static uint64_t getopt_ulonglong(char *optarg)
{
  int rc;
  uint64_t value;
  rc = sscanf(optarg, "0x%llx" SCNxMAX, &value);
  if (rc <= 0)
    rc = sscanf(optarg, "%ul" SCNxMAX, &value);
  //fprintf(stderr, "sscanf() = %d, value = 0x%llx\n", rc, (unsigned long long)value);
  return value;
}

static volatile void *userbar_addr = NULL;
static volatile void *bypassbar_addr = NULL;

#define FPGA_690T_REG(i, r)			(bypassbar_addr + (0x20*(i)) + (r) + 0x100)
#define FPGA_690T_START_REG(i)		(bypassbar_addr + (8*((i) - 1)) + 0x100)
#define FPGA_690T_FINISH_REG(i)		(bypassbar_addr + (8*((i) - 1)) + 0x104)

static inline uint32_t fly_reg_rd(volatile uint32_t *reg) 
{
	msleep(100);
	return *reg;
}	

static inline void fly_reg_wr(uint32_t val, volatile uint32_t *reg) 
{
	msleep(100);
	reg[0] = val;
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
			timespec_sub(&ts_end, &ts_start);
			fprintf(stderr, "Spend %ld.%09ld seconds (total).\n", ts_end.tv_sec, ts_end.tv_nsec);
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

int main(int argc, char* argv[])
{
	int cmd_opt;
	char *device = FLYSLICE_DEV_FILE;
	uint32_t id = FPGA_ID_DEFAULT;
	char *image_flp = NULL;
	char *cfg_flp = NULL;
	char *res_flp = NULL;
	char *fpga_flp = NULL;
	int rc = 55;

	while ((cmd_opt = getopt_long(argc, argv, "hi:f:c:t:s:o:", long_opts, NULL)) != -1) {
		switch (cmd_opt) {
			case 0:
			/* long option */
				break;
			/* image file name */
			case 'f':
				image_flp = strdup(optarg);
				break;
			/* config file name */
			case 'c':
				cfg_flp = strdup(optarg);
				break;
			/* get id number of 690T */
			case 'i':
				id = getopt_integer(optarg);
				break;
			/* inference data file name */
			case 'o':
				res_flp = strdup(optarg);
				break;
			/* FPGA image file name */
			case 't':
				fpga_flp = strdup(optarg);
				break;
			/* print usage help and exit */
			case 'h':
				default:
				usage(argv[0]);
				exit(0);
				break;
		}
	}
	fprintf(stderr, "Begin to inference for image file<%s> on device file<%s>\n"
			"====>need to load FPGA on 690T-<%d>? <%s>"
			"====>need to load config? <%s>" 
			"====>need to save result? <%s>\n", 
			image_flp? image_flp:"None", device, id,
			fpga_flp ? fpga_flp : "no",
			cfg_flp ? cfg_flp : "no",
			res_flp ? res_flp : "no");

	userbar_addr = (unsigned int *)_mmap4bar(USER_BAR_PHY_ADDR, USER_BAR_PHY_LEN);
	fprintf(stderr, "Get User BAR access addr = 0x%llx\n", userbar_addr);
	bypassbar_addr = (unsigned int *)_mmap4bar(BYPASS_BAR_PHY_ADDR, BYPASS_BAR_PHY_LEN);
	fprintf(stderr, "Get Bypass BAR access addr = 0x%llx\n", bypassbar_addr);
	if (!userbar_addr || !bypassbar_addr) {
		fprintf(stderr, "ERROR: mapping FPGA BAR space failed.\n");
		exit(0);
	}

	/* Loading FPGA bit image file into 690t */
	if (fpga_flp) {
		if (fpga_690t_load(device, fpga_flp, id) < 0)
			goto FAILED_EXIT;
		fpga_trigger((uint32_t *)(userbar_addr + 0x100), (uint32_t *)(userbar_addr + 0x104), 0);
		fprintf(stderr, "Running 690T-%d Done!\n", id);
	}
	
	/* Loading Configuration on 690t */
	if (cfg_flp) {
		if (yolo_cfg_load(device, cfg_flp, id) < 0)
			goto FAILED_EXIT;
		fprintf(stderr, "[Linc]>>>>>> Trigger start at 0x%lx, wait for finish at 0x%lx\n", FPGA_690T_START_REG(id), FPGA_690T_FINISH_REG(id));
		fpga_trigger((uint32_t *)FPGA_690T_START_REG(id), (uint32_t *)FPGA_690T_FINISH_REG(id), 0);
		fprintf(stderr, "Loading Configuration on 690T-%d Done!\n", id);
	}
	/* Running image inference on 690t */
	if (image_flp) {
		rc = yolo_test(device, image_flp, res_flp, id);
		// fprintf(stderr, "Test on 690T-%d Done!\n", id);
	}
FAILED_EXIT:
	munmap((void*)userbar_addr, USER_BAR_PHY_LEN);
	munmap((void*)bypassbar_addr, BYPASS_BAR_PHY_LEN);
	return rc;
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
	
	//__set_cap();
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

#define PAGE_SIZE	4096UL
#define MAP_MASK	(PAGE_SIZE - 1)
static void *_mmap4bar(off_t phy_addr, int size)
{	
	int fd;
	void *map_base, *virt_addr;	

	if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
		return NULL;
	}

    fprintf(stderr, "[fly_mmap4bar]>>>>> Map %d bytes on Phy address 0x%llx\n", size, phy_addr);
    
    /* Map*/
    map_base = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phy_addr & ~MAP_MASK);
    if (map_base == (void *) -1) {
		return NULL;
	}
    fprintf(stderr, "BAR Memory mapped at address %p.\n", map_base);
    
    virt_addr = map_base + (phy_addr & MAP_MASK);
	close(fd);
	return virt_addr;
}

static int _fly_import(const char* dev, unsigned long long offset, 
						const unsigned char *data, int len) 
{
	int fd, rc;
	struct fly_ioc_import ioc;
	
	//__set_cap();
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

static int fpga_690t_load(char *devicename, const char *filename, int id)
{
	int rc = 0, size = 0;
	FILE *bin_fd = fopen(filename, "rb");
	struct stat st;
	char *allocated = NULL;
	char *buffer = NULL;

	if (!bin_fd) {
		fprintf(stderr, "ERROR: can't open the file(%s)\n", filename);
		return -1;
	}
	rc = stat(filename, &st);
	size = st.st_size;
	if (!size) {
		fprintf(stderr, "ERROR: file<%d> data size is zero.\n", filename);
		return -1;
	}
	fprintf(stderr, "Downloading FPGA bin file<%s>[size=0x%lx] on 690T-%d.\n", filename, size, id);
	posix_memalign((void **)&allocated, 4096/*alignment*/, size + 4096);	
	assert(allocated);
	buffer = allocated;
	memset(buffer , 0, size);
	rc = fread(buffer, 1, size, bin_fd);
	fclose(bin_fd);
	if (rc != size) {
		fprintf(stderr, "ERROR: read data size error, rc=%d.\n", rc);
		free(allocated);
		return -1;
	}
	rc = _fly_import(devicename, FPGA_690T_BIN_DDR_OFFSET, buffer, size);
	if (rc < 0) {
		fprintf(stderr, "ERROR: downloading FPGA bin file error (rc=%d).\n", rc);
		return rc;
	}
	msleep(10);
	free(allocated);
	barrier();
	fly_reg_wr(FPGA_690T_BIN_DDR_OFFSET, userbar_addr + 0x108);
	barrier();
	fprintf(stderr, "[Linc]>>>> 108=0x%x\n", fly_reg_rd(userbar_addr + 0x108));
	fly_reg_wr(size, userbar_addr + 0x10C);
	barrier();
	fprintf(stderr, "[Linc]>>>> 10C=0x%x\n", fly_reg_rd(userbar_addr + 0x10C));
	fly_reg_wr(id, userbar_addr + 0x110);
	barrier();
	fprintf(stderr, "[Linc]>>>> 110=0x%x\n", fly_reg_rd(userbar_addr + 0x110));
	fprintf(stderr, "Downloading 690T-%d Done!\n", id);
	return rc;
}

static int yolo_cfg_load(char *devicename, const char *filename, int id)
{
	int rc = 0, size = 0;
	FILE *cfg_fd = fopen(filename, "rb");
	struct stat st;
	char *allocated = NULL;
	char *buffer = NULL;

	if (!cfg_fd) {
		fprintf(stderr, "ERROR: can't open the file(%s)\n", filename);
		return -1;
	}
	rc = stat(filename, &st);
	size = st.st_size;
	if (!size) {
		fprintf(stderr, "ERROR: file<%d> data size is zero.\n", filename);
		return -1;
	}
	fprintf(stderr, "Downloading Config data file<%s>[size=0x%lx] on 690T.\n", filename, size);
	posix_memalign((void **)&allocated, 4096/*alignment*/, size + 4096);	
	assert(allocated);
	buffer = allocated;
	memset(buffer , 0, size);
	rc = fread(buffer, 1, size, cfg_fd);
	fclose(cfg_fd);
	if (rc != size) {
		fprintf(stderr, "ERROR: read data size error, rc=%d.\n", rc);
		free(allocated);
		return -1;
	}
	rc = _fly_import(devicename, FPGA_690T_CFG_DDR_OFFSET, buffer, size);
	if (rc < 0) {
		fprintf(stderr, "ERROR: downloading FPGA bin file error (rc=%d).\n", rc);
		return rc;
	}
	free(allocated);
	barrier();
	fly_reg_wr(FPGA_690T_CFG_DDR_OFFSET, FPGA_690T_REG(id, 0x04));
	barrier();
	//fprintf(stderr, "[Linc]>>>> 0x%x=0x%x\n", FPGA_690T_REG(id, 0x04), fly_reg_rd(FPGA_690T_REG(id, 0x04)));
	
	//fly_reg_wr(size, FPGA_690T_REG(id, 0x0C)); // size = 0x8000000?
	fly_reg_wr(0x8000000, FPGA_690T_REG(id, 0x0C)); // size = 0x8000000?
	barrier();
	fprintf(stderr, "[Linc]>>>> 0x%x=0x%x\n", FPGA_690T_REG(id, 0x0C), fly_reg_rd(FPGA_690T_REG(id, 0x0C)));
	
	fly_reg_wr(1, FPGA_690T_REG(id, 0x10)); // set cfg loading mode
	barrier();
	//fprintf(stderr, "[Linc]>>>> 0x%x=0x%x\n", FPGA_690T_REG(id, 0x10), fly_reg_rd(FPGA_690T_REG(id, 0x10)));
	fly_reg_wr(0x10000000, FPGA_690T_REG(id, 0x14)); // set cfg loading address
	barrier();
	//fprintf(stderr, "[Linc]>>>> 0x%x=0x%x\n", FPGA_690T_REG(id, 0x14), fly_reg_rd(FPGA_690T_REG(id, 0x14)));
	fprintf(stderr, "Downloading Config Data Done!\n");
	return rc;
}

char inferTitle[256];

char *create_json(struct fly_yolo_obj* objects, int num)
{
    char *string = NULL;
    size_t index = 0;

    cJSON *objJson = cJSON_CreateObject();
    if (objJson == NULL)
    {
        goto end;
    }

    cJSON *detectedObjs = cJSON_CreateArray();
    if (detectedObjs == NULL)
    {
        goto end;
    }
    cJSON_AddItemToObject(objJson, inferTitle, detectedObjs);

    for (index = 0; index < num; ++index)
    {
        cJSON *obj = cJSON_CreateObject();
        if (obj == NULL)
        {
            goto end;
        }
        cJSON_AddItemToArray(detectedObjs, obj);

        cJSON *box_x0 = cJSON_CreateNumber(objects[index].box_x0);
        if (box_x0 == NULL)
        {
            goto end;
        }
        cJSON_AddItemToObject(obj, "box_x0", box_x0);

        cJSON *box_x1 = cJSON_CreateNumber(objects[index].box_x1);
        if (box_x1 == NULL)
        {
            goto end;
        }
        cJSON_AddItemToObject(obj, "box_x1", box_x1);

        cJSON *box_y0 = cJSON_CreateNumber(objects[index].box_y0);
        if (box_y0 == NULL)
        {
            goto end;
        }
        cJSON_AddItemToObject(obj, "box_y0", box_y0);

        cJSON *box_y1 = cJSON_CreateNumber(objects[index].box_y1);
        if (box_y1 == NULL)
        {
            goto end;
        }
        cJSON_AddItemToObject(obj, "box_y1", box_y1);

        cJSON *prob = cJSON_CreateNumber(objects[index].prob);
        if (prob == NULL)
        {
            goto end;
        }
        cJSON_AddItemToObject(obj, "prob", prob);

        cJSON *cls = cJSON_CreateNumber(objects[index].cls);
        if (cls == NULL)
        {
            goto end;
        }
        cJSON_AddItemToObject(obj, "cls", cls);

        cJSON *seq = cJSON_CreateNumber(objects[index].seq);
        if (seq == NULL)
        {
            goto end;
        }
        cJSON_AddItemToObject(obj, "seq", seq);

        cJSON *boxes = cJSON_CreateNumber(objects[index].boxes);
        if (boxes == NULL)
        {
            goto end;
        }
        cJSON_AddItemToObject(obj, "boxes", boxes);        

    }

    string = cJSON_PrintUnformatted(objJson);
    if (string == NULL)
    {
        fprintf(stderr, "Failed to print monitor.\n");
    }

end:
    cJSON_Delete(objJson);
    return string;
}

/**
 * \brief dump data from FPGA memory.
 *
 * \param file is the name of dump file to save 
 * \param size is dump data size
 * \param id is the core id to dump
 *
 */
int yolo_test(char *devicename, const char *image_path, const char *out_path, int id)
{
	int i, rc = 0, size = 0;
	uint32_t obj_num = 0, dect;
	FILE *img_fd = fopen(image_path, "rb");
	FILE *dump_fd;
	struct stat st;
	struct fly_yolo_obj *obj;
	char *allocated = NULL;
	char *buffer = NULL;

	if (!img_fd) {
		fprintf(stderr, "ERROR: can't open the file(%s)\n", image_path);
		return -2;
	}
	rc = stat(image_path, &st);
	size = st.st_size;
	if (!size) {
		fprintf(stderr, "ERROR: file<%d> data size is zero.\n", image_path);
		return -3;
	}
	// fprintf(stderr, "Downloading Image data file<%s>[size=0x%lx] on 690T.\n", image_path, size);
	posix_memalign((void **)&allocated, 4096/*alignment*/, size + 4096);	
	assert(allocated);
	buffer = allocated;
	memset(buffer , 0, size);
	rc = fread(buffer, 1, size, img_fd);
	fclose(img_fd);
	if (rc != size) {
		fprintf(stderr, "ERROR: read data size error, rc=%d.\n", rc);
		free(allocated);
		return -4;
	}
	rc = _fly_import(devicename, FPGA_690T_IMG_DDR_OFFSET, buffer, size);
	if (rc < 0)
		fprintf(stderr, "ERROR: downloading FPGA bin file error (rc=%d).\n", rc);
	msleep(10);
	free(allocated);
	allocated = NULL;
	barrier();

	fly_reg_wr(FPGA_690T_IMG_DDR_OFFSET, FPGA_690T_REG(id, 0x00));
	barrier();
	// fprintf(stderr, "[Linc]>>>> 0x%lx=0x%x\n", FPGA_690T_REG(id, 0x00), fly_reg_rd(FPGA_690T_REG(id, 0x00)));
	
	fly_reg_wr(size, FPGA_690T_REG(id, 0x08));
	barrier();
	// fprintf(stderr, "[Linc]>>>> 0x%lx=0x%x\n", FPGA_690T_REG(id, 0x08), fly_reg_rd(FPGA_690T_REG(id, 0x08)));
	
	fly_reg_wr(2, FPGA_690T_REG(id, 0x10)); // set image loading mode
	barrier();
	// fprintf(stderr, "[Linc]>>>> 0x%lx=0x%x\n", FPGA_690T_REG(id, 0x10), fly_reg_rd(FPGA_690T_REG(id, 0x10)));
	
	fly_reg_wr(0x10000000, FPGA_690T_REG(id, 0x14)); // set image loading address
	barrier();
	// fprintf(stderr, "[Linc]>>>> 0x%lx=0x%x\n", FPGA_690T_REG(id, 0x14), fly_reg_rd(FPGA_690T_REG(id, 0x14)));
	// fprintf(stderr, "Downloading Image Data Done!\n");
	// fprintf(stderr, "[Linc]>>>>>> Trigger start at 0x%lx, wait for finish at 0x%lx\n", FPGA_690T_START_REG(id), FPGA_690T_FINISH_REG(id));
	fpga_trigger((uint32_t *)FPGA_690T_START_REG(id), (uint32_t *)FPGA_690T_FINISH_REG(id), 0);
	// fprintf(stderr, "Image inference Done!\n");
	barrier();

	dect = 0;
	do {
		dect = fly_reg_rd(FPGA_690T_REG(id, 0x18));
	} while (!dect);

	obj_num = fly_reg_rd(FPGA_690T_REG(id, 0x1C));	
	barrier();
	// fprintf(stderr, "Detect %d objects in this image. saving result? <%s>\n",
	// 	obj_num, out_path ? "yes":"no");
	
	size = YOLO_ONE_OBJ_SIZE * obj_num;
	if (!size) {
		fprintf(stderr, "No object is detected!\n");
		return -5;
	}	
	posix_memalign((void **)&allocated, 4096/*alignment*/, size + 4096);	
	assert(allocated);
	buffer = allocated;
	memset(buffer , 0, size);
	rc = _fly_dump(devicename, 0x10000000, buffer, size, 0);
	if (rc < 0)
		fprintf(stderr, "ERROR: dump result error(rc=%d).\n", rc);
	
	obj = (struct fly_yolo_obj *)buffer;
	for (i = 0; i < obj_num; i++) {
		fprintf(stderr, "=========  Object-<%d> ============\n", i);
		fprintf(stderr, "Object Class is %d\n", obj[i].cls);
		fprintf(stderr, "Object Probability is %f\n", obj[i].prob);
		fprintf(stderr, "Box position is X-(%d, %d),  Y-(%d. %d)\n", obj[i].box_x0, obj[i].box_x1, obj[i].box_y0, obj[i].box_y1);
		fprintf(stderr, "===================================\n");
	}
	
	if (obj_num > 0 ) {
		sprintf(inferTitle, "detected %d objects", obj_num);
		char* outString = create_json(obj, obj_num);
		write(STDOUT_FILENO, outString, strlen(outString));
	}

	if (out_path) {
		fprintf(stderr, "Saving result file<%s>.\n", out_path);
		dump_fd = fopen(out_path, "wb+");
		if (!dump_fd) {
			fprintf(stderr, "ERROR: can't open the file(%s)\n", out_path);
			return;
		}		
		fwrite(buffer, size, 1, dump_fd);
		fclose(dump_fd);
	}
	free(allocated);
	return 0;
}
