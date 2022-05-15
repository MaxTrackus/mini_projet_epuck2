#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

/**
* @brief   return the width of the object to central unit
*
* @return			width in pixels
*/
uint16_t get_lineWidth(void);

/**
* @brief   return the flag storing if the object is seen or not
*
* @return			boolean staticFoundLine
*/
bool get_staticFoundLine(void);

/**
* @brief   return the position of the object on the camera
*
* @return			position in pixel
*/
uint16_t get_line_position(void);

/**
* @brief   Init CaptureImage and ProcessImage threads
*/
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
