typedef struct xSense_t
{
    TickType_t xTimeStamp;
    uint16_t ulValue;
}xSense_t;


/**
 * @brief Initialize Mailbox Queue
 */
static QueueHandle_t vQueueInit(void);


void vUpdateQueue(QueueHandle_t Queue, float ulNewValue);

/**
 * @brief 
 *        Read Mailbox Queue
 * @param pxData Pointer to Struct of type xSense_t for contents to be copied
 * @param Queue  Pointer to struct of type xSense_T to be read from
 */
static void vReadQueue(xSense_t *pxData, QueueHandle_t Queue);