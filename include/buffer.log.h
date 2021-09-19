/*
 * buffer.h
 *
 *  Created on: Фев 20, 2020
 *      Author: jet
 */

#ifndef LOG_BUF_H
#define LOG_BUF_H 100

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "logger.h"
/**
 * @defgroup logBuf_Macros
 * @brief    Library defines
 * @{
 */

#define logBuf_INITIALIZED     0x01 /*!< Buffer initialized flag */
#define logBuf_MALLOC          0x02 /*!< Buffer uses malloc for memory */
#define logBuf_OVER            0x04

/* Custom allocation and free functions if needed */
#ifndef LIB_ALLOC_FUNC
#define LIB_ALLOC_FUNC         malloc
#endif
#ifndef LIB_FREE_FUNC
#define LIB_FREE_FUNC          free
#endif

#ifndef logBuf_FAST
#define logBuf_FAST            1
#endif

/**
 * @}
 */

/**
 * @defgroup logBuf_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  Buffer structure
 */
typedef struct _logBuf_t {
  uint32_t Size;           /*!< Size of buffer in units of sizeof( sCanMsg ), DO NOT MOVE OFFSET, 0 */
  sLogRec * In;             /*!< Input pointer to save next struct sCanMsg, DO NOT MOVE OFFSET, 1 */
  sLogRec * Out;            /*!< Output pointer to read next value, DO NOT MOVE OFFSET, 2 */
  sLogRec * Buffer;         /*!< Pointer to buffer data array, DO NOT MOVE OFFSET, 3 */
  uint8_t Flags;           /*!< Flags for buffer, DO NOT MOVE OFFSET, 4 */
  void* UserParameters;    /*!< Pointer to user value if needed */
} logBuf_t;

/**
 * @}
 */

/**
 * @defgroup logBuf_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes buffer structure for work
 * @param  *Buffer: Pointer to @ref logBuf_t structure to initialize
 * @param  Size: Size of buffer in units of bytes
 * @param  *BufferPtr: Pointer to array for buffer storage. Its length should be equal to @param Size parameter.
 *           If NULL is passed as parameter, @ref malloc will be used to allocate memory on heap.
 * @retval Buffer initialization status:
 *            - 0: Buffer initialized OK
 *            - > 0: Buffer initialization error. Malloc has failed with allocation
 */
uint8_t logBuf_Init(logBuf_t* Buffer, sLogRec * BufferPtr, uint16_t Size );

/**
 * @brief  Free memory for buffer allocated using @ref malloc
 * @note   This function has sense only if malloc was used for dynamic allocation
 * @param  *Buffer: Pointer to @ref logBuf_t structure
 * @retval None
 */
void logBuf_Free(logBuf_t* Buffer);

/**
 * @brief  Writes data to buffer
 * @param  *Buffer: Pointer to @ref logBuf_t structure
 * @param  *Data: Pointer to data to be written
 * @param  count: Number of elements of type unsigned char to write
 * @retval Number of elements written in buffer
 */
uint16_t logBuf_Write(logBuf_t* Buffer, sLogRec * Data, uint16_t count);
uint16_t logBuf_WriteMsg(logBuf_t* Buffer, sLogRec * msg);

/**
 * @brief  Reads data from buffer
 * @param  *Buffer: Pointer to @ref logBuf_t structure
 * @param  *Data: Pointer to data where read values will be stored
 * @param  count: Number of elements of type unsigned char to read
 * @retval Number of elements read from buffer
 */
uint16_t logBuf_Read(logBuf_t* Buffer, sLogRec * Data, uint16_t count);
uint16_t logBuf_ReadMsg(logBuf_t* Buffer, sLogRec * msg);

/**
 * @brief  Check: buffer is free?
 * @param  *Buffer: Pointer to @ref logBuf_t structure
 * @retval  1 - buffer is free
 *          0 - buffer isn't free
 */
static inline bool logBuf_IsFree(logBuf_t* Buffer) {
  return  Buffer->In == Buffer->Out;

}


/**
 * @brief  Gets number of free elements in buffer
 * @param  *Buffer: Pointer to @ref logBuf_t structure
 * @retval Number of free elements in buffer
 */
uint16_t logBuf_GetFree(logBuf_t* Buffer);

/**
 * @brief  Gets number of elements in buffer
 * @param  *Buffer: Pointer to @ref logBuf_t structure
 * @retval Number of elements in buffer
 */
uint16_t logBuf_GetFull(logBuf_t* Buffer);

/**
 * @brief  Resets (clears) buffer pointers
 * @param  *Buffer: Pointer to @ref logBuf_t structure
 * @retval None
 */
void logBuf_Reset(logBuf_t* Buffer);

/**
 * @brief  Checks if specific element value is stored in buffer
 * @param  *Buffer: Pointer to @ref logBuf_t structure
 * @param  uint8_t Element: Element to check
 * @retval Status of element:
 *            -  < 0: Element was not found
 *            - >= 0: Element found, location in buffer is returned
 *                   Ex: If value 1 is returned, it means 1 read from buffer and your element will be returned
 */
int16_t logBuf_FindElement(logBuf_t* Buffer, sLogRec * Element);

/**
 * @brief  Sets string delimiter character when reading from buffer as string
 * @param  Buffer: Pointer to @ref logBuf_t structure
 * @param  StrDel: Character as string delimiter
 * @retval None
 */
#define logBuf_SetStringDelimiter(Buffer, StrDel)  ((Buffer)->StringDelimiter = (StrDel))

/**
 * @brief  Checks if character exists in location in buffer
 * @param  *Buffer: Pointer to @ref logBuf_t structure
 * @param  pos: Position in buffer, starting from 0
 * @param  *element: Pointer to save value at desired position to be stored into
 * @retval Check status:
 *            - 0: Buffer is not so long as position desired
 *            - > 0: Position to check was inside buffer data size
 */
int8_t logBuf_CheckElement(logBuf_t* Buffer, uint16_t pos, uint8_t* element);

/**
 * @}
 */

/**
 * @}
 */


#endif /* LOG_BUF_H_ */
