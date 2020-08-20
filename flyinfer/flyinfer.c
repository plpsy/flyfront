#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "cJSON.h"
#include "flyslice_cnn_api.h"

//create a monitor with a list of supported resolutions
//NOTE: Returns a heap allocated string, you are required to free it after use.
char *create_monitor(void)
{
    const unsigned int resolution_numbers[3][2] = {
        {1280, 720},
        {1920, 1080},
        {3840, 2160}
    };
    char *string = NULL;
    cJSON *name = NULL;
    cJSON *resolutions = NULL;
    cJSON *resolution = NULL;
    cJSON *width = NULL;
    cJSON *height = NULL;
    size_t index = 0;

    cJSON *monitor = cJSON_CreateObject();
    if (monitor == NULL)
    {
        goto end;
    }

    name = cJSON_CreateString("Awesome 4K");
    if (name == NULL)
    {
        goto end;
    }
    /* after creation was successful, immediately add it to the monitor,
     * thereby transferring ownership of the pointer to it */
    cJSON_AddItemToObject(monitor, "name", name);

    resolutions = cJSON_CreateArray();
    if (resolutions == NULL)
    {
        goto end;
    }
    cJSON_AddItemToObject(monitor, "resolutions", resolutions);

    for (index = 0; index < (sizeof(resolution_numbers) / (2 * sizeof(int))); ++index)
    {
        resolution = cJSON_CreateObject();
        if (resolution == NULL)
        {
            goto end;
        }
        cJSON_AddItemToArray(resolutions, resolution);

        width = cJSON_CreateNumber(resolution_numbers[index][0]);
        if (width == NULL)
        {
            goto end;
        }
        cJSON_AddItemToObject(resolution, "width", width);

        height = cJSON_CreateNumber(resolution_numbers[index][1]);
        if (height == NULL)
        {
            goto end;
        }
        cJSON_AddItemToObject(resolution, "height", height);
    }

    string = cJSON_PrintUnformatted(monitor);
    if (string == NULL)
    {
        fprintf(stderr, "Failed to print monitor.\n");
    }

end:
    cJSON_Delete(monitor);
    return string;
}

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
    cJSON_AddItemToObject(objJson, "detected objects", detectedObjs);

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

int main()
{
    fly_object objects[16];
    objects[0].box_x0 = 1;
    objects[0].box_x1 = 2;
    objects[0].box_y0 = 3;
    objects[0].box_y1 = 4;
    objects[0].boxes = 2;
    objects[0].cls = 1;
    objects[0].prob = 2.1;
    objects[0].seq = 0;

    objects[1].box_x0 = 2;
    objects[1].box_x1 = 2;
    objects[1].box_y0 = 3;
    objects[1].box_y1 = 4;
    objects[1].boxes = 2;
    objects[1].cls = 1;
    objects[1].prob = 2.4;
    objects[1].seq = 0;

	char* outString = create_json(objects, 2);
	write(STDOUT_FILENO, outString, strlen(outString)); 
	return 0;
}