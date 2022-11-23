#ifndef SYS_RESOURCE_H
#define SYS_RESOURCE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/**
 * @brief Unit of message between tasks for both temperature and flow rate
 * 
 */
typedef struct xSense_t
{
    TickType_t xTimeStamp;
    uint16_t ulValue;
} xSense_t;


/**
 * @brief Initialize Mailbox Queue
 */
QueueHandle_t vQueueInit(void);


void vUpdateQueue(QueueHandle_t Queue, uint16_t ulNewValue);

/**
 * @brief 
 *        Read Mailbox Queue
 * @param pxData Pointer to Struct of type xSense_t for contents to be copied
 * @param Queue  Pointer to struct of type xSense_T to be read from
 */
void vReadQueue(xSense_t *pxData, QueueHandle_t Queue);

#endif