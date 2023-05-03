/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <portmacro.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

/*** Generic Status Return Values ***/
#define SUCCESS                   0x00 //!< SUCCESS
#define FAILURE                   0x01 //!< Failure

#define TRUE  1    // true
#define FALSE 0    // false


QueueHandle_t Queue_create(uint32_t QueueLen, uint32_t msgSize);
uint8_t Queue_enqueue(QueueHandle_t msgQueue, char *pMsg);
uint8_t* Queue_dequeue(QueueHandle_t msgQueue);
uint8_t Queue_empty(QueueHandle_t msgQueue);
