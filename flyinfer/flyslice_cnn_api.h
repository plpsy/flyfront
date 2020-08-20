/*********************************************************
Copyright (C ) 2020 Flyslice Technologies Co.,Ltd
Author : Licheng Gu
**********************************************************/
#ifndef _FLYSLICE_CNN_API_H_
#define _FLYSLICE_CNN_API_H_
#include <inttypes.h>
#include <sys/types.h>

/**
 * @brief The definition of the error code
 */
typedef enum _FLYCODE
{
    FLYCODE_OK = 0,          // OK
    FLYCODE_INVALID_DATA,
	FLYCODE_LOADING_ERROR,
	FLYCODE_START_FPGA_FAILED,
    // TODO: Please supplement more error codes whenever the implementation requires.

} FLYCODE;

/**
 * @brief Loading program of the specified FPGA chipset  
 * 
 * @param id The 690t FPGA index, 0-based, [1..4]
 * @param data The data buffer address
 * @param len The data buffer length
 * @return Refer to the error code definition
 */
FLYCODE loading_690t(int id, const uint8_t *data, int len);

/**
 * @brief Running program of the specified FPGA chipset  
 * 
 * @param id The 690t FPGA index, 0-based, [1..4]
 * @return Refer to the error code definition
 */
FLYCODE running_690t(int id);

/**
 * @brief Loading CNN configuration&weights data for the specified FPGA chipset  
 * 
 * @param id The 690t FPGA index, 0-based, [1..4]
 * @param data The data buffer address
 * @param len The data buffer length
 * @return Refer to the error code definition
 */
FLYCODE loading_cnn_cfg(int id, const uint8_t *data, int len);

/**
 * @brief The definition of the detect object
 */
typedef struct __attribute__((__packed__))  _FLY_CNN_OBJ
{
	uint16_t box_x0;
	uint16_t box_x1;
	uint16_t box_y0;
	uint16_t box_y1;
	float prob;
	uint16_t cls;
	uint8_t seq;
	uint8_t boxes;
} fly_object;

/**
 * @brief Doing CNN inference for the specified image data 
 * 
 * @param id The 690t FPGA index, 0-based, [1..4]
 * @param img_data The image data buffer address
 * @param img_len The image data buffer length
 * @param result [OUT] The result data buffer address
 * @param objs [OUT] The detected object number
 * @return Refer to the error code definition
 */
FLYCODE fpga_inference(int id, const uint8_t *img_data, int img_len, uint8_t *result, int *objs);

#endif /* _FLYSLICE_CNN_API_H_ */
