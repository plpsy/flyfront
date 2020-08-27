/*********************************************************
Copyright (C ) 2020 Flyslice Technologies Co.,Ltd
Author : Licheng Gu
**********************************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <time.h>
#include <string.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/time.h>

#include "flyslice_cnn_api.h"
#include "cJSON.h"

#define FLY_YOLO_MAX_OBJS		256
#define SIZE_DEFAULT (4096)
#define FPGA_ID_DEFAULT (1)

static struct option const long_opts[] =
{
  {"config", required_argument, NULL, 'c'},
  {"inference", required_argument, NULL, 'f'},
  {"id", required_argument, NULL, 'i'},
  {"fpga", required_argument, NULL, 't'},
  {"help", no_argument, NULL, 'h'},
  {0, 0, 0, 0}
};
  
static uint32_t getopt_integer(char *optarg)
{
  int rc;
  uint32_t value;
  rc = sscanf(optarg, "0x%x", &value);
  if (rc <= 0)
    rc = sscanf(optarg, "%ul", &value);
  return value;
}

static uint64_t getopt_ulonglong(char *optarg)
{
  int rc;
  uint64_t value;
  rc = sscanf(optarg, "0x%llx" SCNxMAX, &value);
  if (rc <= 0)
    rc = sscanf(optarg, "%ul" SCNxMAX, &value);
  return value;
}

static void usage(const char* name)
{
	int i = 0;
	printf("%s\n\n", name);
	printf("usage: %s [OPTIONS]\n\n", name);
	printf("Test one image data on FPGA running YOLOVx\n\n");

	printf("  -%c (--%s) yolo config data file for loading.\n", long_opts[i].val, long_opts[i].name); i++;
	printf("  -%c (--%s) image data file for inference.\n", long_opts[i].val, long_opts[i].name); i++;
	printf("  -%c (--%s) the number of 690t for starting, default is %d.\n", long_opts[i].val, long_opts[i].name, FPGA_ID_DEFAULT); i++; 
	printf("  -%c (--%s) 690T FPGA BIT file for loading.\n", long_opts[i].val, long_opts[i].name); i++;
	printf("  -%c (--%s) print usage help and exit\n", long_opts[i].val, long_opts[i].name); i++;
}

char inferTitle[256];

char *create_json(fly_object* objects, int num)
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

int main(int argc, char* argv[])
{
	int cmd_opt;
	int i, rc = 0, size = 0, objs = 0;
	uint32_t id = FPGA_ID_DEFAULT;
	char *image_flp = NULL;
	char *cfg_flp = NULL;
	char *res_flp = NULL;
	char *fpga_flp = NULL;
	FILE *fd = NULL;
	struct stat st;
	char *allocated = NULL;
	char *buffer = NULL;
	fly_object *result = NULL;
		
	while ((cmd_opt = getopt_long(argc, argv, "hi:f:c:t:", long_opts, NULL)) != -1) {
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

	if (fpga_flp) {
		fd = fopen(fpga_flp, "rb");
		if (!fd) {
			printf("ERROR: can't open the file(%s)\n", fpga_flp);
			return -1;
		}
		rc = stat(fpga_flp, &st);
		size = st.st_size;
		if (!size) {
			printf("ERROR: file<%d> data size is zero.\n", fpga_flp);
			return -1;
		}
		printf("Downloading FPGA bin file<%s>[size=%d] on 690T-%d.\n", fpga_flp, size, id);
		posix_memalign((void **)&allocated, 4096/*alignment*/, size + 4096);	
		buffer = allocated;
		memset(buffer , 0, size);
		rc = fread(buffer, 1, size, fd);
		rc = loading_690t(id, buffer, size);
		rc = running_690t(id);
		fclose(fd);
	}

	if (cfg_flp) {
		fd = fopen(cfg_flp, "rb");
		if (!fd) {
			printf("ERROR: can't open the file(%s)\n", cfg_flp);
			return -1;
		}
		rc = stat(cfg_flp, &st);
		size = st.st_size;
		if (!size) {
			printf("ERROR: file<%d> data size is zero.\n", cfg_flp);
			return -1;
		}
		printf("Downloading YOLO configuration file<%s>[size=%d] on 690T-%d.\n", cfg_flp, size, id);
		posix_memalign((void **)&allocated, 4096/*alignment*/, size + 4096);	
		buffer = allocated;
		memset(buffer , 0, size);
		rc = fread(buffer, 1, size, fd);
		rc = loading_cnn_cfg(id, buffer, size);
		fclose(fd);
	}

	if (image_flp) {
		fd = fopen(image_flp, "rb");
		if (!fd) {
			printf("ERROR: can't open the file(%s)\n", image_flp);
			return -1;
		}
		printf("open image file ok\n");
		rc = stat(image_flp, &st);
		size = st.st_size;
		if (!size) {
			printf("ERROR: file<%s> data size is zero.\n", image_flp);
			return -1;
		}
		printf("inference file<%s>[size=%d] on 690T-%d.\n", image_flp, size, id);
		posix_memalign((void **)&allocated, 4096/*alignment*/, size + 4096);
		buffer = allocated;
		result = (fly_object*)malloc(FLY_YOLO_MAX_OBJS*sizeof(fly_object));
		memset(buffer , 0, size);
		rc = fread(buffer, 1, size, fd);
		rc = fpga_inference(id, (uint8_t*)buffer, size, (uint8_t*)result, &objs);
		fclose(fd);
		if (objs) {
			printf("We have detected %d objects.\n", objs);
			for (i = 0; i < objs; i++)
				printf("Object-<%d>: X-[%u, %u], Y-[%u, %u], class-<%d>, prob-<%f>.\n", 
							i, result[i].box_x0, result[i].box_x1, result[i].box_y0, result[i].box_y1,
							result[i].cls, result[i].prob);
		} else {
			printf("There is no objects detected.\n");
		}

		if (objs) {
			char* outString = create_json(result, objs);
			write(STDOUT_FILENO, outString, strlen(outString));
		}
	}
	return 0;
}