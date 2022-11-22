/**
 * @brief Initialize Mailbox Queue
 */
static QueueHandle_t vQueueInit(void)
{   
    return xQueueCreate(1, sizeof(xSense_t));
}

void vUpdateQueue(QueueHandle_t Queue, float ulNewValue)
{
    xSense_t xData;
    xData.ulValue = ulNewValue;
    xData.xTimeStamp = pdTICKS_TO_MS(xTaskGetTickCount());
    xQueueOverwrite(Queue, &xData);
}

/**
 * @brief 
 *        Read Mailbox Queue
 * @param pxData Pointer to Struct of type xSense_t for contents to be copied
 * @param Queue  Pointer to struct of type xSense_T to be read from
 */
static void vReadQueue(xSense_t *pxData, QueueHandle_t Queue)
{
    xQueuePeek(Queue, pxData, portMAX_DELAY);
}