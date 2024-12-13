//
// --------------------------------------------------------------------------
//  Gurux Ltd
//
//
//
// Filename:        $HeadURL:  $
//
// Version:         $Revision:  $,
//                  $Date:  $
//                  $Author: $
//
// Copyright (c) Gurux Ltd
//
//---------------------------------------------------------------------------

#include "communication.h"

#include "errorcodes.h"
#include "gxkey.h"
#include "gxmem.h"
#include "converters.h"
#include "cosem.h"
//#include "driver_init.h"
//#include "utils.h"
#include <stdlib.h>
#include "dlms_client.h"
#include "serial_modbus.h"
#include "usart.h"
#include "tick.h"
#include "udp_protocol.h"
#include "message_queue.h"
#include "dlms_log.h"
#include "shutdown.h"
#include "mbus_protocol.h"
#include "circular_buffer.h"
#include "dlms_client_table.h"

//variantArray user_variantArray[10];

extern UART_HandleTypeDef huart2;

//variantArray * com_getUserVariantArray( uint8_t pos )
//{
//	return &user_variantArray[pos];
//}

//Returns current time.
//If you are not using operating system you have to implement this by yourself.
//Reason for this is that all compilers's or HWs don't support time at all.
void time_now(gxtime* value)
{

}

int com_readSerialPort(
    connection *connection,
    unsigned char eop)
{
    //Read reply data.
    int pos;
    (void)pos;
    unsigned char eopFound = 0;
    unsigned short lastReadIndex = 0;
    (void)lastReadIndex;
    uint8_t data[500] = {0};
    uint16_t i = 0;
    int index = 0;
    uint32_t timeout = 15500;
    uint32_t tickstart = HAL_GetTick();
	/* Clears the FIFO UART RX buffer*/
	__HAL_UART_FLUSH_DRREGISTER(&huart2);

	do
    {
		//io_read(connection->io, connection->data.data, 1);
		HAL_StatusTypeDef ret = HAL_UART_Receive(&huart2, &data[i], 1, timeout);//500//15500
		timeout = 500;
//		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> ret:%d...\n", (int)Tick_Get(SECONDS), (int)ret);
		if (ret == HAL_OK)
    	{
//    		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Data i:%d...\n", (int)Tick_Get(SECONDS), (int)i);
    		bb_setUInt8(&connection->data, data[i++]);
    	}
    	else if (ret == HAL_TIMEOUT)
    	{
    		eopFound = 1;
    		/* No answer from the meter*/
    		if (0 == i)
    		{
    			return DLMS_ERROR_CODE_RECEIVE_FAILED;
    		}
//    		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DataEnd i:%d...\n", (int)Tick_Get(SECONDS), (int)i);
    		break;
    	}
    	/* hw not ready */
    	else
    	{
    		return DLMS_ERROR_CODE_RECEIVE_FAILED;
    	}
        //connection->data.size += 1;
        //Search eop.
//        if (connection->data.size > 5)
//        {
//            //Some optical strobes can return extra bytes.
//            for (pos = connection->data.size - 1; pos != lastReadIndex; --pos)
//            {
//                if (connection->data.data[pos] == eop)
//                {
//                    eopFound = 1;
//                    break;
//                }
//            }
//            lastReadIndex = pos;
//        }
    } while ((eopFound == 0) && ((HAL_GetTick() - tickstart) < 20000));
	if (connection->trace == GX_TRACE_LEVEL_VERBOSE)
    {
        char* hex = hlp_bytesToHex(connection->data.data + *(&index), connection->data.size - *(&index));
        if (*(&index) == 0)
        {
//            printf("\nRX:\t %s", hex);
            LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> RX:\t%s\n", (int)Tick_Get(SECONDS), hex);
        }
        else
        {
//            printf(" %s", hex);
            LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> %s\n", (int)Tick_Get(SECONDS), hex);
        }
        gxfree(hex);
        *(&index) = connection->data.size;
    }
    return DLMS_ERROR_CODE_OK;
}

// Read DLMS Data frame from the device.
int readDLMSPacket(
    connection *connection,
    gxByteBuffer* data,
    gxReplyData* reply)
{
	char* hex;
	int ret = DLMS_ERROR_CODE_SEND_FAILED;//DLMS_ERROR_CODE_OK;
	uint32_t tickstart = HAL_GetTick();
    if (data->size == 0)
    {
        return DLMS_ERROR_CODE_OK;
    }
    reply->complete = 0;
    connection->data.size = 0;
    connection->data.position = 0;
    if (connection->trace == GX_TRACE_LEVEL_VERBOSE)
    {
        hex = bb_toHexString(data);
        LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> TX:\t%s\n", (int)Tick_Get(SECONDS), hex);
        gxfree(hex);
    }
	//io_write(connection->io, data->data, data->size);
    ENABLE_RS485();
    DE_TX_RS485();
    MX_USART2_UART_DLMS_Init();
    HAL_Delay(1);
    HAL_UART_Transmit(&huart2, data->data, data->size, 0x0FFF);
    DE_RX_RS485();
    //Loop until packet is complete.
    do
    {
        if (com_readSerialPort(connection, 0x7E) != 0)
        {
            return DLMS_ERROR_CODE_SEND_FAILED;
        }
        ret = cl_getData(&connection->settings, &connection->data, reply);
        if (ret != 0 && ret != DLMS_ERROR_CODE_FALSE)
        {
        	DISABLE_RS485();
            break;
        }
    } while ((reply->complete == 0) && ((HAL_GetTick() - tickstart) < 20000));
    DISABLE_RS485();
    return ret;
}

int com_readDataBlock(
    connection *connection,
    message* messages,
    gxReplyData* reply)
{
    gxByteBuffer rr;
    int pos, ret = DLMS_ERROR_CODE_OK;
    //If there is no data to send.
    if (messages->size == 0)
    {
        return DLMS_ERROR_CODE_OK;
    }
    bb_init(&rr);
    //Send data.
    for (pos = 0; pos != messages->size; ++pos)
    {
        //Send data.
        if ((ret = readDLMSPacket(connection, messages->data[pos], reply)) != DLMS_ERROR_CODE_OK)
        {
            return ret;
        }
        //Check is there errors or more data from server
        while (reply_isMoreData(reply))
        {
            if ((ret = cl_receiverReady(&connection->settings, reply->moreData, &rr)) != DLMS_ERROR_CODE_OK)
            {
                bb_clear(&rr);
                return ret;
            }
            if ((ret = readDLMSPacket(connection, &rr, reply)) != DLMS_ERROR_CODE_OK)
            {
                bb_clear(&rr);
                return ret;
            }
            bb_clear(&rr);
        }
    }
    return ret;
}

//Close connection to the meter.
int com_close(
    connection *connection)
{
    int ret = DLMS_ERROR_CODE_OK;
    gxReplyData reply;
    message msg;
    reply_init(&reply);
    mes_init(&msg);
    if ((ret = cl_releaseRequest(&connection->settings, &msg)) != 0 ||
        (ret = com_readDataBlock(connection, &msg, &reply)) != 0)
    {
        //Show error but continue close.
        LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Release request failed. Error = %d; Parsing RLRQ...\n", (int)Tick_Get(SECONDS), ret);
    }
    else
    {
        LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> RLRQ OK.\n", (int)Tick_Get(SECONDS));
    }
    reply_clear(&reply);
    mes_clear(&msg);
    reply_init(&reply);
    mes_init(&msg);
    if ((ret = cl_disconnectRequest(&connection->settings, &msg)) != 0 ||
        (ret = com_readDataBlock(connection, &msg, &reply)) != 0)
    {
        //Show error but continue close.
        LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Closing connection failed. Error = %d; Parsing RLRE\n", (int)Tick_Get(SECONDS), ret);
    }
    else
    {
        LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Closing connection OK.\n", (int)Tick_Get(SECONDS));
    }
    reply_clear(&reply);
    mes_clear(&msg);
//    bb_clear(&(connection->data));
//    cl_clear(&connection->settings);
    return ret;
}

//Initialize connection to the meter.
int com_initializeConnection(
    connection *connection)
{
    int ret = DLMS_ERROR_CODE_OK;
    message messages;
    gxReplyData reply;

    mes_init(&messages);
    reply_init(&reply);

    //Get meter's send and receive buffers size.
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Sending SNRM...\n", (int)Tick_Get(SECONDS));
    if ((ret = cl_snrmRequest(&connection->settings, &messages)) != 0 ||
        (ret = com_readDataBlock(connection, &messages, &reply)) != 0 ||
        (ret = cl_parseUAResponse(&connection->settings, &reply.data)) != 0)
    {
    	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Error = %d; Parsing UA...\n", (int)Tick_Get(SECONDS), ret);
        mes_clear(&messages);
        reply_clear(&reply);
        return ret;
    }
    LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> UARE OK.\n", (int)Tick_Get(SECONDS));
    mes_clear(&messages);
    reply_clear(&reply);
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Sending AARQ...\n", (int)Tick_Get(SECONDS));
    if ((ret = cl_aarqRequest(&connection->settings, &messages)) != 0 ||
        (ret = com_readDataBlock(connection, &messages, &reply)) != 0 ||
        (ret = cl_parseAAREResponse(&connection->settings, &reply.data)) != 0)
    {
        mes_clear(&messages);
        reply_clear(&reply);
    	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Error = %d; Parsing AARQ...\n", (int)Tick_Get(SECONDS), ret);
        if (ret == DLMS_ERROR_CODE_APPLICATION_CONTEXT_NAME_NOT_SUPPORTED)
        {
            return ret;
        }
        return ret;
    }
    LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> AARE OK.\n", (int)Tick_Get(SECONDS));
    mes_clear(&messages);
    reply_clear(&reply);
    int ret_buff = 0;
    if (connection->settings.maxPduSize == 0xFFFF)
    {
        ret_buff = con_initializeBuffers(connection, connection->settings.maxPduSize);
    }
    else
    {
        //Allocate 50 bytes more because some meters count this wrong and send few bytes too many.
        ret_buff = con_initializeBuffers(connection, 50 + connection->settings.maxPduSize);
    }
    if (DLMS_ERROR_CODE_OUTOFMEMORY == ret_buff)
    {
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> ERROR!! OUTOFMEMORY when Initialize buffers.\n", (int)Tick_Get(SECONDS));
    }
    // Get challenge Is HLS authentication is used.
    if (connection->settings.authentication > DLMS_AUTHENTICATION_LOW)
    {
        if ((ret = cl_getApplicationAssociationRequest(&connection->settings, &messages)) != 0 ||
            (ret = com_readDataBlock(connection, &messages, &reply)) != 0 ||
            (ret = cl_parseApplicationAssociationResponse(&connection->settings, &reply.data)) != 0)
        {
            mes_clear(&messages);
            reply_clear(&reply);
        	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Error = %d; Parsing get challenge...\n", (int)Tick_Get(SECONDS), ret);
            return ret;
        }

        mes_clear(&messages);
        reply_clear(&reply);
    }
    return DLMS_ERROR_CODE_OK;
}

//Report error on output;
void com_reportError(const char* description,
    gxObject* object,
    unsigned char attributeOrdinal, int ret)
{
    char ln[25];
    char type[30];

    hlp_getLogicalNameToString(object->logicalName, ln);
    obj_typeToString(object->objectType, type);
//    printf("%s %s %s:%d %s\r\n", description, type, ln, attributeOrdinal, hlp_getErrorMessage(ret));
	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> %s %s %s:%d %s\r\n", (int)Tick_Get(SECONDS), description, type, ln, attributeOrdinal, hlp_getErrorMessage(ret));

}

//Report success on output;
void com_reportSuccess(const char* description,
    gxObject* object,
    unsigned char attributeOrdinal,
	dlmsVARIANT* dataType,
	int ret)
{
    char ln[25] = {'\0'};
    char type[30] = {'\0'};
	char value[128] = {'\0'};
    uint32_t data_value = 0, print_val = 0;

    hlp_getLogicalNameToString(object->logicalName, ln);
    obj_typeToString(object->objectType, type);
#if 0
    DLMS_DATA_TYPE_NONE = 0,
     DLMS_DATA_TYPE_BOOLEAN = 3,
     DLMS_DATA_TYPE_BIT_STRING = 4,
     DLMS_DATA_TYPE_INT32 = 5,
     DLMS_DATA_TYPE_UINT32 = 6,
     DLMS_DATA_TYPE_OCTET_STRING = 9,
     DLMS_DATA_TYPE_STRING = 10,
     DLMS_DATA_TYPE_BINARY_CODED_DESIMAL = 13,
     DLMS_DATA_TYPE_STRING_UTF8 = 12,
     DLMS_DATA_TYPE_INT8 = 15,
     DLMS_DATA_TYPE_INT16 = 16,
     DLMS_DATA_TYPE_UINT8 = 17,
     DLMS_DATA_TYPE_UINT16 = 18,
     DLMS_DATA_TYPE_INT64 = 20,
     DLMS_DATA_TYPE_UINT64 = 21,
     DLMS_DATA_TYPE_ENUM = 22,
     DLMS_DATA_TYPE_FLOAT32 = 23,
     DLMS_DATA_TYPE_FLOAT64 = 24,
     DLMS_DATA_TYPE_DATETIME = 25,
     DLMS_DATA_TYPE_DATE = 26,
     DLMS_DATA_TYPE_TIME = 27,
     DLMS_DATA_TYPE_ARRAY = 1,
     DLMS_DATA_TYPE_STRUCTURE = 2,
     DLMS_DATA_TYPE_COMPACT_ARRAY = 19,
     DLMS_DATA_TYPE_BYREF = 0x80
#endif
    switch (dataType->vt)
    {
    case DLMS_DATA_TYPE_NONE:
    	break;
    case DLMS_DATA_TYPE_INT8:
    	data_value = 1;
    	print_val  = dataType->cVal;
    	itoa(dataType->cVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_UINT8:
    	data_value = 1;
    	print_val  = dataType->bVal;
    	itoa(dataType->bVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_INT16:
    	data_value = 1;
    	print_val  = dataType->iVal;
    	itoa(dataType->iVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_UINT16:
    	data_value = 1;
    	print_val  = dataType->uiVal;
    	itoa(dataType->uiVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_INT32:
    	data_value = 1;
    	print_val  = dataType->lVal;
    	itoa(dataType->lVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_UINT32:
    	data_value = 1;
    	print_val  = dataType->ulVal;
    	itoa(dataType->ulVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_INT64:
    	data_value = 1;
    	print_val  = dataType->llVal;
    	(void)print_val;
//    	itoa(dataType->llVal, value, 10);
    	i64tostrn_a(dataType->llVal, value, sizeof(value));
    	break;
    case DLMS_DATA_TYPE_UINT64:
    	data_value = 1;
    	print_val  = dataType->ullVal;
//    	itoa(dataType->ullVal, value, 10);
    	u64tostrn_a(dataType->ullVal, value, sizeof(value));
    	break;

    case DLMS_DATA_TYPE_STRUCTURE:
    {
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_DATA_TYPE_STRUCTURE %s %s %s:%d = %s -> %s\r\n", (int)Tick_Get(SECONDS), description, type, ln, attributeOrdinal, dataType->strVal->data, hlp_getErrorMessage(ret));
    	char *data = NULL;
    	data = gxmalloc(64*sizeof(uint8_t));
    	memset(data,'\0',64*sizeof(uint8_t));
    	obj_toString(object, &data);
//    	strcat(data,"\0");
    	dlms_client_set_obis_value(data, strlen(data), attributeOrdinal);
    	gxfree(data);
    	data = 	NULL;
    }
    	break;

    case DLMS_DATA_TYPE_STRING:
    case DLMS_DATA_TYPE_STRING_UTF8:
    case DLMS_DATA_TYPE_OCTET_STRING:
    {
    	/* Add a null byte to get the right string */
    	uint32_t err = bb_setUInt8(dataType->strVal, '\0');
    	if (err != DLMS_ERROR_CODE_INVALID_PARAMETER)
    	{
    		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_DATA_TYPE_STRING %s %s %s:%d = %s -> %s\r\n", (int)Tick_Get(SECONDS), description, type, ln, attributeOrdinal, dataType->strVal->data, hlp_getErrorMessage(ret));
    		dlms_client_set_obis_value((char *)dataType->strVal->data, strlen((char *)dataType->strVal->data), attributeOrdinal);
    	}
    }
    break;
    case DLMS_DATA_TYPE_BIT_STRING:
    {
    	char *data_arr = NULL;
    	obj_toString(object, &data_arr);
    	strcat(data_arr,"\0");
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_DATA_TYPE_ARRAY -> %s\r\n", (int)Tick_Get(SECONDS), data_arr);
    	dlms_client_set_obis_value((char *)(data_arr + 16), strlen((char *)(data_arr + 16)), attributeOrdinal);
    	gxfree(data_arr);
    	data_arr = 	NULL;
    }
    break;
    case DLMS_DATA_TYPE_ARRAY:
    {
    	char *data_arr = NULL;
    	obj_toString(object, &data_arr);
    	strcat(data_arr,"\0");
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_DATA_TYPE_ARRAY -> %s\r\n", (int)Tick_Get(SECONDS), data_arr);
    	gxfree(data_arr);
    	data_arr = 	NULL;
    }
    	break;
    default:
    	break;
    }
    if ( 1 == data_value )
    {
    	data_value = 0;
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_DATA_TYPE_INT %s %s %s:%d = %s -> %s\r\n", (int)Tick_Get(SECONDS), description, type, ln, attributeOrdinal, value, hlp_getErrorMessage(ret));
    	if (( 2 == attributeOrdinal ) && (DLMS_OBJECT_TYPE_DATA == object->objectType))
    	{
    		dlms_client_append_obis_value(value, attributeOrdinal);
    	}
//    	else
//    	{
//    		dlms_client_set_obis_value(value, strlen(value), attributeOrdinal);
//    	}
    }
}

//Report success on output;
void com_reportSuccess2(const char* description,
    gxObject* object,
    unsigned char attributeOrdinal,
	dlmsVARIANT* dataType,
	int ret)
{
    char ln[25] = {'\0'};
    char type[30] = {'\0'};
	char value[128] = {'\0'};
    uint32_t data_value = 0, print_val = 0;

    (void)data_value;

    hlp_getLogicalNameToString(object->logicalName, ln);
    obj_typeToString(object->objectType, type);

    switch (dataType->vt)
    {
    case DLMS_DATA_TYPE_NONE:
    	break;
    case DLMS_DATA_TYPE_INT8:
    	data_value = 1;
    	print_val  = dataType->cVal;
    	itoa(dataType->cVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_UINT8:
    	data_value = 1;
    	print_val  = dataType->bVal;
    	itoa(dataType->bVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_INT16:
    	data_value = 1;
    	print_val  = dataType->iVal;
    	itoa(dataType->iVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_UINT16:
    	data_value = 1;
    	print_val  = dataType->uiVal;
    	itoa(dataType->uiVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_INT32:
    	data_value = 1;
    	print_val  = dataType->lVal;
    	itoa(dataType->lVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_UINT32:
    	data_value = 1;
    	print_val  = dataType->ulVal;
    	itoa(dataType->ulVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_INT64:
    	data_value = 1;
    	print_val  = dataType->llVal;
    	(void)print_val;
//    	itoa(dataType->llVal, value, 10);
    	i64tostrn_a(dataType->llVal, value, sizeof(value));
    	break;
    case DLMS_DATA_TYPE_UINT64:
    	data_value = 1;
    	print_val  = dataType->ullVal;
//    	itoa(dataType->ullVal, value, 10);
    	u64tostrn_a(dataType->ullVal, value, sizeof(value));
    	break;

    case DLMS_DATA_TYPE_STRUCTURE:
    {
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_DATA_TYPE_STRUCTURE %s %s %s:%d = %s -> %s\r\n", (int)Tick_Get(SECONDS), description, type, ln, attributeOrdinal, dataType->strVal->data, hlp_getErrorMessage(ret));
    	char *data = NULL;
    	data = gxmalloc(64*sizeof(uint8_t));
    	memset(data,'\0',64*sizeof(uint8_t));
    	obj_toString(object, &data);
//    	strcat(data,"\0");
    	dlms_client_set_obis_value(data, strlen(data), attributeOrdinal);
    	gxfree(data);
    	data = 	NULL;
    }
    	break;

    case DLMS_DATA_TYPE_STRING:
    case DLMS_DATA_TYPE_STRING_UTF8:
    case DLMS_DATA_TYPE_OCTET_STRING:
    case DLMS_DATA_TYPE_BIT_STRING:
    {
    	char value[64];
    	char *data_arr = NULL;
    	memset(value,0,64);
    	sprintf(value,"%s","1,");
    	obj_toString(object, &data_arr);
    	strcat(data_arr,"\0");
    	strcat(value, data_arr + 16);
    	strcat(value, ",255");
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_DATA_TYPE_ARRAY -> %s\r\n", (int)Tick_Get(SECONDS), value);
    	dlms_client_set_obis_value((char *)value, strlen((char *)value), attributeOrdinal);
    	gxfree(data_arr);
    	data_arr = 	NULL;
    }
    break;
    case DLMS_DATA_TYPE_ARRAY:
    {
    	char *data_arr = NULL;
    	obj_toString(object, &data_arr);
    	strcat(data_arr,"\0");
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_DATA_TYPE_ARRAY -> %s\r\n", (int)Tick_Get(SECONDS), data_arr);
    	gxfree(data_arr);
    	data_arr = 	NULL;
    }
    	break;
    default:
    	break;
    }
}

//Report success on output;
void com_reportSuccess3(const char* description,
    gxObject* object,
    unsigned char attributeOrdinal,
	dlmsVARIANT* dataType,
	int ret)
{
    char ln[25] = {'\0'};
    char type[30] = {'\0'};
	char value[128] = {'\0'};
    uint32_t data_value = 0, print_val = 0;

    (void)data_value;

    hlp_getLogicalNameToString(object->logicalName, ln);
    obj_typeToString(object->objectType, type);

    switch (dataType->vt)
    {
    case DLMS_DATA_TYPE_NONE:
    	break;
    case DLMS_DATA_TYPE_INT8:
    	data_value = 1;
    	print_val  = dataType->cVal;
    	itoa(dataType->cVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_UINT8:
    	data_value = 1;
    	print_val  = dataType->bVal;
    	itoa(dataType->bVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_INT16:
    	data_value = 1;
    	print_val  = dataType->iVal;
    	itoa(dataType->iVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_UINT16:
    	data_value = 1;
    	print_val  = dataType->uiVal;
    	itoa(dataType->uiVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_INT32:
    	data_value = 1;
    	print_val  = dataType->lVal;
    	itoa(dataType->lVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_UINT32:
    	data_value = 1;
    	print_val  = dataType->ulVal;
    	itoa(dataType->ulVal, value, 10);
    	break;
    case DLMS_DATA_TYPE_INT64:
    	data_value = 1;
    	print_val  = dataType->llVal;
    	(void)print_val;
//    	itoa(dataType->llVal, value, 10);
    	i64tostrn_a(dataType->llVal, value, sizeof(value));
    	break;
    case DLMS_DATA_TYPE_UINT64:
    	data_value = 1;
    	print_val  = dataType->ullVal;
//    	itoa(dataType->ullVal, value, 10);
    	u64tostrn_a(dataType->ullVal, value, sizeof(value));
    	break;

    case DLMS_DATA_TYPE_STRUCTURE:
    {
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_DATA_TYPE_STRUCTURE %s %s %s:%d = %s -> %s\r\n", (int)Tick_Get(SECONDS), description, type, ln, attributeOrdinal, dataType->strVal->data, hlp_getErrorMessage(ret));
    	char *data = NULL;
    	data = gxmalloc(64*sizeof(uint8_t));
    	memset(data,'\0',64*sizeof(uint8_t));
    	obj_toString(object, &data);
//    	strcat(data,"\0");
    	dlms_client_set_obis_value(data, strlen(data), attributeOrdinal);
    	gxfree(data);
    	data = 	NULL;
    }
    	break;

    case DLMS_DATA_TYPE_STRING:
    case DLMS_DATA_TYPE_STRING_UTF8:
    case DLMS_DATA_TYPE_OCTET_STRING:
    case DLMS_DATA_TYPE_BIT_STRING:
    {
    	char value[64];
//    	char token[14];
    	char no_data[4] = " , ";
    	char *data_arr = NULL;
    	char *str;
    	memset(value,0,64);
    	sprintf(value,"%s","1,");
    	obj_toString(object, &data_arr);
    	strcat(data_arr,"\0");
    	str = strtok( data_arr + 17, "(" );
    	str = strtok( NULL, "*" );
    	if (*str==')')
    	{
    		strcat(value, no_data);
    	}
    	else
    	{
    		strcat(value, str);
			strcat(value, ",");
			str = strtok( NULL, ")" );
			strcat(value, str);
    	}
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_DATA_TYPE_ARRAY -> %s\r\n", (int)Tick_Get(SECONDS), value);
    	dlms_client_set_obis_value((char *)value, strlen((char *)value), attributeOrdinal);
    	gxfree(data_arr);
    	data_arr = 	NULL;
    }
    break;
    case DLMS_DATA_TYPE_ARRAY:
    {
    	char *data_arr = NULL;
    	obj_toString(object, &data_arr);
    	strcat(data_arr,"\0");
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_DATA_TYPE_ARRAY -> %s\r\n", (int)Tick_Get(SECONDS), data_arr);
    	gxfree(data_arr);
    	data_arr = 	NULL;
    }
    	break;
    default:
    	break;
    }
}
//Get Association view.
int com_getAssociationView(connection *connection)
{
    int ret;
    message data;
    gxReplyData reply;
    mes_init(&data);
    reply_init(&reply);
    if ((ret = cl_getObjectsRequest(&connection->settings, &data)) != 0 ||
        (ret = com_readDataBlock(connection, &data, &reply)) != 0 ||
        (ret = cl_parseObjects(&connection->settings, &reply.data)) != 0)
    {
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> ReadObject failed\r\n", (int)Tick_Get(SECONDS));
    }
    mes_clear(&data);
    reply_clear(&reply);
    return ret;
}

//Read object.
int com_read(
    connection *connection,
    gxObject* object,
    unsigned char attributeOrdinal)
{
    int ret;
    message data;
    gxReplyData reply;
    mes_init(&data);
    reply_init(&reply);
    if ((ret = cl_read(&connection->settings, object, attributeOrdinal, &data)) != 0 ||
        (ret = com_readDataBlock(connection, &data, &reply)) != 0 ||
        (ret = cl_updateValue(&connection->settings, object, attributeOrdinal, &reply.dataValue)) != 0)
    {
        com_reportError("ReadObject failed", object, attributeOrdinal, ret);
    }
    else
    {
    	com_reportSuccess("ReadObject success... ", object, attributeOrdinal, &reply.dataValue, ret);
    }
    mes_clear(&data);
    reply_clear(&reply);
    return ret;
}

//Read object.
int com_read2(
    connection *connection,
    gxObject* object,
    unsigned char attributeOrdinal)
{
    int ret;
    message data;
    gxReplyData reply;
    mes_init(&data);
    reply_init(&reply);
    if ((ret = cl_read(&connection->settings, object, attributeOrdinal, &data)) != 0 ||
        (ret = com_readDataBlock(connection, &data, &reply)) != 0 ||
        (ret = cl_updateValue(&connection->settings, object, attributeOrdinal, &reply.dataValue)) != 0)
    {
        com_reportError("ReadObject failed", object, attributeOrdinal, ret);
    }
    else
    {
    	com_reportSuccess2("ReadObject success... ", object, attributeOrdinal, &reply.dataValue, ret);
    }
    mes_clear(&data);
    reply_clear(&reply);
    return ret;
}

//Read object.
int com_read3(
    connection *connection,
    gxObject* object,
    unsigned char attributeOrdinal)
{
    int ret;
    message data;
    gxReplyData reply;
    mes_init(&data);
    reply_init(&reply);
    if ((ret = cl_read(&connection->settings, object, attributeOrdinal, &data)) != 0 ||
        (ret = com_readDataBlock(connection, &data, &reply)) != 0 ||
        (ret = cl_updateValue(&connection->settings, object, attributeOrdinal, &reply.dataValue)) != 0)
    {
        com_reportError("ReadObject failed", object, attributeOrdinal, ret);
    }
    else
    {
    	com_reportSuccess3("ReadObject success... ", object, attributeOrdinal, &reply.dataValue, ret);
    }
    mes_clear(&data);
    reply_clear(&reply);
    return ret;
}

int com_write(
    connection *connection,
    gxObject* object,
    unsigned char attributeOrdinal)
{
    int ret;
    message data;
    gxReplyData reply;
    mes_init(&data);
    reply_init(&reply);
    if ((ret = cl_write(&connection->settings, object, attributeOrdinal, &data)) != 0 ||
        (ret = com_readDataBlock(connection, &data, &reply)) != 0)
    {
        com_reportError("Write failed", object, attributeOrdinal, ret);
    }
    else
    {
    	com_reportSuccess("WriteObject success... ", object, attributeOrdinal, &reply.dataValue, ret);
    }
    mes_clear(&data);
    reply_clear(&reply);
    return ret;
}

int com_method(
    connection *connection,
    gxObject* object,
    unsigned char attributeOrdinal,
    dlmsVARIANT* params)
{
    int ret;
    message messages;
    gxReplyData reply;
    mes_init(&messages);
    reply_init(&reply);
    if ((ret = cl_method(&connection->settings, object, attributeOrdinal, params, &messages)) != 0 ||
        (ret = com_readDataBlock(connection, &messages, &reply)) != 0)
    {
//        printf("Method failed %s\r\n", hlp_getErrorMessage(ret));
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Method failed %s\r\n", (int)Tick_Get(SECONDS), hlp_getErrorMessage(ret));
    }
    mes_clear(&messages);
    reply_clear(&reply);
    return ret;
}

//Read objects.
int com_readList(
    connection *connection,
    gxArray* list)
{
    int pos, ret = DLMS_ERROR_CODE_OK;
    gxByteBuffer bb, rr;
    message messages;
    gxReplyData reply;
    if (list->size != 0)
    {
        mes_init(&messages);
        if ((ret = cl_readList(&connection->settings, list, &messages)) != 0)
        {
            printf("ReadList failed %s\r\n", hlp_getErrorMessage(ret));
        }
        else
        {
            reply_init(&reply);
            bb_init(&rr);
            bb_init(&bb);
            //Send data.
            for (pos = 0; pos != messages.size; ++pos)
            {
                //Send data.
                reply_clear(&reply);
                if ((ret = readDLMSPacket(connection, messages.data[pos], &reply)) != DLMS_ERROR_CODE_OK)
                {
                    break;
                }
                //Check is there errors or more data from server
                while (reply_isMoreData(&reply))
                {
                    if ((ret = cl_receiverReady(&connection->settings, reply.moreData, &rr)) != DLMS_ERROR_CODE_OK ||
                        (ret = readDLMSPacket(connection, &rr, &reply)) != DLMS_ERROR_CODE_OK)
                    {
                        break;
                    }
                    bb_clear(&rr);
                }
                bb_set2(&bb, &reply.data, reply.data.position, -1);
            }
            if (ret == 0)
            {
                ret = cl_updateValues(&connection->settings, list, &bb);
            }
            bb_clear(&bb);
            bb_clear(&rr);
            reply_clear(&reply);
        }
        mes_clear(&messages);
    }
    return ret;
}

int com_readRowsByEntry(
    connection *connection,
    gxProfileGeneric* object,
    unsigned long index,
    unsigned long count)
{
    int ret;
    message data;
    gxReplyData reply;
    uint32_t send_event = 0;
    mes_init(&data);
    reply_init(&reply);
    if ((ret = cl_readRowsByEntry(&connection->settings, object, index, count, &data)) != 0 ||
        (ret = com_readDataBlock(connection, &data, &reply)) != 0 ||
        (ret = cl_updateValue(&connection->settings, (gxObject*)object, 2, &reply.dataValue)) != 0)
    {
//    	udp_protocol_set_on_billing_profile(0);
//    	udp_protocol_set_on_event_profile(0);
    	LOGLIVE(LEVEL_2, "LOGLIVE> %d ReadObject by Entry failed %s\r\n", (int)Tick_Get(SECONDS), hlp_getErrorMessage(ret));
    }
    else
    {
    	LOGLIVE(LEVEL_2, "LOGLIVE> %d ReadObject OK %s\r\n", (int)Tick_Get(SECONDS), hlp_getErrorMessage(ret));
    	CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_generic_profile_frame_type());
    	read_task_t con_read_task = (read_task_t)CircularBuffer_Read(dlms_client_get_read_msg_queue());
		if ((con_read_task >= DLMS_READ_CMD_INST_PROF_1) && (con_read_task <= DLMS_READ_CMD_EVENT_LOG_PROF))
		{
			con_read_task -= DLMS_READ_CMD_INST_PROF_1;
		}
		if ((con_read_task == DLMS_READ_CMD_EVENT_LOG_PROF) || con_read_task == DLMS_READ_EVENT_LOG_PROF)
		{
			if (1 == dlms_log_store_raw_eventsprofile(reply.data.data, reply.data.size, 2 * reply.data.size))
			{
				reply.data.size = 0;
				if ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) == (DLMS_WAIT_FOR_SEND_EVENT_LOG_PROF))
				{
//					CircularBuffer_Get(dlms_client_get_send_msg_queue());
				}
			}
			else
			{
				send_event = 1;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> NEW Event Log Meter TO SEND:%d. \r\n",
				        						(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
//				CircularBuffer_Put(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_EVENT_LOG_PROF);
			}
		}
		else
		{
			dlms_log_store_raw_loadprofile(reply.data.data, reply.data.size, 2 * reply.data.size);
		}
//    	dlms_client_table_write_generic_profile(con_dlms_get_curr_device(), dlms_client_get_client(), dlms_client_get_client_generic_profile(), dlms_client_get_generic_profile_frame_type());
//    	dlms_client_table_read_client_generic_profile(con_dlms_get_curr_device(), dlms_client_get_client(),  dlms_client_get_client_generic_profile(), dlms_client_get_generic_profile_frame_type());
    	if ( reply.data.size > 0 )
    	{
//			if (( CircularBuffer_Read(dlms_client_get_send_msg_queue()) >= (DLMS_WAIT_FOR_SEND_LOAD_PROF_1) )
//			 && ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) <= (DLMS_WAIT_FOR_SEND_EVENT_LOG_PROF) ))
    		if ( ( 1 == send_event ) || ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) == (DLMS_WAIT_FOR_SEND_INST_PROF_1 + dlms_client_get_frame_type_last_write()) ) )
    		{
    			shutdown_setInitTelitModule(1);
    			message_queue_write(SEND_MODBUS_SENSOR);
    			if ( 0 == send_event )
    			{
    				CircularBuffer_Get(dlms_client_get_send_msg_queue());
    			}
    			send_event = 0;
    			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Put Send Queue.\r\n", (int)Tick_Get( SECONDS ));
    			CircularBuffer_Put(dlms_client_get_send_msg_queue(), dlms_client_get_frame_type_last_write());
    		}
    	}
    }
    mes_clear(&data);
    reply_clear(&reply);
    return ret;
}

///////////////////////////////////////////////////////////////////////////////////
//char hexa[6000];
int com_readRowsByRange(
    connection *connection,
    gxProfileGeneric* object,
    struct tm* start,
    struct tm* end)
{
    int ret;
    message data;
    gxReplyData reply;
    uint32_t start_time, end_time;

    start_time = mktime(start);
    end_time   = mktime(end);
    mes_init(&data);
    reply_init(&reply);
    if ((ret = cl_readRowsByRange(&connection->settings, object, start_time, end_time, &data)) != 0 ||
        (ret = com_readDataBlock(connection, &data, &reply)) != 0 ||
        (ret = cl_updateValue(&connection->settings, (gxObject*)object, 2, &reply.dataValue)) != 0)
    {
//    	udp_protocol_set_on_billing_profile(0);
//    	udp_protocol_set_on_event_profile(0);
    	LOGLIVE(LEVEL_2, "LOGLIVE> %d ReadObject by Range failed %s\r\n", (int)Tick_Get(SECONDS), hlp_getErrorMessage(ret));
    }
    else
    {
    	LOGLIVE(LEVEL_2, "LOGLIVE> %d ReadObject OK %s\r\n", (int)Tick_Get(SECONDS), hlp_getErrorMessage(ret));
    	CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_generic_profile_frame_type());
    	read_task_t con_read_task = (read_task_t)CircularBuffer_Read(dlms_client_get_read_msg_queue());
		if ((con_read_task >= DLMS_READ_CMD_INST_PROF_1) && (con_read_task <= DLMS_READ_CMD_EVENT_LOG_PROF))
		{
			con_read_task -= DLMS_READ_CMD_INST_PROF_1;
		}
		if ((con_read_task == DLMS_READ_CMD_EVENT_LOG_PROF) || con_read_task == DLMS_READ_EVENT_LOG_PROF)
		{
			if (1 == dlms_log_store_raw_eventsprofile(reply.data.data, reply.data.size, 2 * reply.data.size))
			{
				reply.data.size = 0;
				if ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) == (DLMS_WAIT_FOR_SEND_EVENT_LOG_PROF))
				{
//					CircularBuffer_Get(dlms_client_get_send_msg_queue());
				}
				else
				{
//					CircularBuffer_Put(dlms_client_get_send_msg_queue(), DLMS_SEND_EVENT_LOG_PROF);
				}
			}
			else
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> NEW Event Log Meter TO SEND:%d. \r\n",
				        						(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
				CircularBuffer_Put(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_EVENT_LOG_PROF);
			}
		}
		else
		{
			dlms_log_store_raw_loadprofile(reply.data.data, reply.data.size, 2 * reply.data.size);
		}

//    	dlms_client_table_write_generic_profile(con_dlms_get_curr_device(), dlms_client_get_client(), dlms_client_get_client_generic_profile(), dlms_client_get_generic_profile_frame_type());
//    	dlms_client_table_read_client_generic_profile(con_dlms_get_curr_device(), dlms_client_get_client(),  dlms_client_get_client_generic_profile(), dlms_client_get_generic_profile_frame_type());
//    	udp_protocol_set_on_demand_command(1);
    	if ( reply.data.size > 0 )
    	{
//			if (( CircularBuffer_Read(dlms_client_get_send_msg_queue()) >= (DLMS_WAIT_FOR_SEND_LOAD_PROF_1) )
//			 && ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) <= (DLMS_WAIT_FOR_SEND_EVENT_LOG_PROF) ))
			if ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) == (DLMS_WAIT_FOR_SEND_INST_PROF_1 + dlms_client_get_frame_type_last_write()) )
    		{
    			shutdown_setInitTelitModule(1);
    			message_queue_write(SEND_MODBUS_SENSOR);
    			CircularBuffer_Get(dlms_client_get_send_msg_queue());
    			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Put Send Queue.\r\n", (int)Tick_Get( SECONDS ));
    			CircularBuffer_Put(dlms_client_get_send_msg_queue(), dlms_client_get_frame_type_last_write());
    		}
    	}
    }
    mes_clear(&data);
    reply_clear(&reply);
    return ret;
}

//Read object.
int com_read_profile_object(
    connection *connection,
    gxObject* object,
    unsigned char attributeOrdinal)
{
    int ret;
    message data;
    gxReplyData reply;
    uint32_t send_event = 0;
    mes_init(&data);
    reply_init(&reply);
    if ((ret = cl_read(&connection->settings, object, attributeOrdinal, &data)) != 0 ||
        (ret = com_readDataBlock(connection, &data, &reply)) != 0 ||
        (ret = cl_updateValue(&connection->settings, object, attributeOrdinal, &reply.dataValue)) != 0)
    {
    	LOGLIVE(LEVEL_2, "LOGLIVE> %d ReadObject by Range failed %s\r\n", (int)Tick_Get(SECONDS), hlp_getErrorMessage(ret));
    }
    else
    {
    	LOGLIVE(LEVEL_2, "LOGLIVE> %d ReadObject OK %s\r\n", (int)Tick_Get(SECONDS), hlp_getErrorMessage(ret));
    	CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_generic_profile_frame_type());
    	read_task_t con_read_task = (read_task_t)CircularBuffer_Read(dlms_client_get_read_msg_queue());
		if ((con_read_task >= DLMS_READ_CMD_INST_PROF_1) && (con_read_task <= DLMS_READ_CMD_EVENT_LOG_PROF))
		{
			con_read_task -= DLMS_READ_CMD_INST_PROF_1;
		}
		if ((con_read_task == DLMS_READ_CMD_EVENT_LOG_PROF) || con_read_task == DLMS_READ_EVENT_LOG_PROF)
		{
			if (1 == dlms_log_store_raw_eventsprofile(reply.data.data, reply.data.size, 2 * reply.data.size))
			{
				reply.data.size = 0;
				if ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) == (DLMS_WAIT_FOR_SEND_EVENT_LOG_PROF))
				{
//					CircularBuffer_Get(dlms_client_get_send_msg_queue());
				}
			}
			else
			{
				send_event = 1;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> NEW Event Log Meter TO SEND:%d. \r\n",
				        						(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
//				CircularBuffer_Put(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_EVENT_LOG_PROF);
			}
		}
		else
		{
			dlms_log_store_raw_loadprofile(reply.data.data, reply.data.size, 2 * reply.data.size);
		}
//    	dlms_client_table_write_generic_profile(con_dlms_get_curr_device(), dlms_client_get_client(), dlms_client_get_client_generic_profile(), dlms_client_get_generic_profile_frame_type());
//    	dlms_client_table_read_client_generic_profile(con_dlms_get_curr_device(), dlms_client_get_client(),  dlms_client_get_client_generic_profile(), dlms_client_get_generic_profile_frame_type());
    	if ( reply.data.size > 0 )
    	{
//			if (( CircularBuffer_Read(dlms_client_get_send_msg_queue()) >= (DLMS_WAIT_FOR_SEND_LOAD_PROF_1) )
//			 && ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) <= (DLMS_WAIT_FOR_SEND_EVENT_LOG_PROF) ))
    		if ( (1 == send_event)||( CircularBuffer_Read(dlms_client_get_send_msg_queue()) == (DLMS_WAIT_FOR_SEND_INST_PROF_1 + dlms_client_get_frame_type_last_write()) ))
    		{
    			shutdown_setInitTelitModule(1);
    			message_queue_write(SEND_MODBUS_SENSOR);
    			if (0 == send_event)
    			{
    			CircularBuffer_Get(dlms_client_get_send_msg_queue());
    			}
    			send_event = 0;
    			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Put Send Queue.\r\n", (int)Tick_Get( SECONDS ));
    			CircularBuffer_Put(dlms_client_get_send_msg_queue(), dlms_client_get_frame_type_last_write());
    		}
    	}
    }
    mes_clear(&data);
    reply_clear(&reply);
    return ret;
}

///////////////////////////////////////////////////////////////////////////////////
//Read scalers and units. They are static so they are read only once.
int com_readScalerAndUnits(
    connection *connection)
{
    gxObject* obj;
    int ret, pos;
    objectArray objects;
    gxArray list;
    gxObject* object;
    DLMS_OBJECT_TYPE types[] = { DLMS_OBJECT_TYPE_EXTENDED_REGISTER, DLMS_OBJECT_TYPE_REGISTER, DLMS_OBJECT_TYPE_DEMAND_REGISTER };
    oa_init(&objects);
    //Find registers and demand registers and read them.
    ret = oa_getObjects2(&connection->settings.objects, types, 3, &objects);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    if ((connection->settings.negotiatedConformance & DLMS_CONFORMANCE_MULTIPLE_REFERENCES) != 0)
    {
        arr_init(&list);
        //Try to read with list first. All meters do not support it.
        for (pos = 0; pos != connection->settings.objects.size; ++pos)
        {
            ret = oa_getByIndex(&connection->settings.objects, pos, &obj);
            if (ret != DLMS_ERROR_CODE_OK)
            {
                oa_empty(&objects);
                arr_clear(&list);
                return ret;
            }
            if (obj->objectType == DLMS_OBJECT_TYPE_REGISTER ||
                obj->objectType == DLMS_OBJECT_TYPE_EXTENDED_REGISTER)
            {
                arr_push(&list, key_init(obj, (void*)3));
            }
            else if (obj->objectType == DLMS_OBJECT_TYPE_DEMAND_REGISTER)
            {
                arr_push(&list, key_init(obj, (void*)4));
            }
        }
        ret = com_readList(connection, &list);
        arr_clear(&list);
    }
    //If read list failed read items one by one.
    if (ret != 0)
    {
        for (pos = 0; pos != objects.size; ++pos)
        {
            ret = oa_getByIndex(&objects, pos, &object);
            if (ret != DLMS_ERROR_CODE_OK)
            {
                oa_empty(&objects);
                return ret;
            }
            ret = com_read(connection, object, object->objectType == DLMS_OBJECT_TYPE_DEMAND_REGISTER ? 4 : 3);
            if (ret != DLMS_ERROR_CODE_OK)
            {
                oa_empty(&objects);
                return ret;
            }
        }
    }
    //Do not clear objects list because it will free also objects from association view list.
    oa_empty(&objects);
    return ret;
}

///////////////////////////////////////////////////////////////////////////////////
//Read profile generic columns. They are static so they are read only once.
int com_readProfileGenericColumns(
    connection *connection)
{
    int ret, pos;
    objectArray objects;
    gxObject* object;
    oa_init(&objects);
    ret = oa_getObjects(&connection->settings.objects, DLMS_OBJECT_TYPE_PROFILE_GENERIC, &objects);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        oa_empty(&objects);
        return ret;
    }
    for (pos = 0; pos != objects.size; ++pos)
    {
        ret = oa_getByIndex(&objects, pos, &object);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            break;
        }
        ret = com_read(connection, object, 3);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            break;
        }
    }
    //Do not clear objects list because it will free also objects from association view list.
    oa_empty(&objects);
    return ret;
}

///////////////////////////////////////////////////////////////////////////////////
//Read profile generics rows.
int com_readProfileGenerics(
    connection *connection)
{
    gxtime startTime, endTime;
    int ret, pos;
    char str[50];
    char ln[25];
    char* data = NULL;
    gxByteBuffer ba;
    objectArray objects;
    gxProfileGeneric* pg;
    oa_init(&objects);
    ret = oa_getObjects(&connection->settings.objects, DLMS_OBJECT_TYPE_PROFILE_GENERIC, &objects);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        //Do not clear objects list because it will free also objects from association view list.
        oa_empty(&objects);
        return ret;
    }
    bb_init(&ba);
    for (pos = 0; pos != objects.size; ++pos)
    {
        ret = oa_getByIndex(&objects, pos, (gxObject**)&pg);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            //Do not clear objects list because it will free also objects from association view list.
            oa_empty(&objects);
            return ret;
        }
        //Read entries in use.
        ret = com_read(connection, (gxObject*)pg, 7);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            if (connection->trace > GX_TRACE_LEVEL_OFF)
            {
                printf("Failed to read object %s %s attribute index %d\r\n", str, ln, 7);
            }
            //Do not clear objects list because it will free also objects from association view list.
            oa_empty(&objects);
            return ret;
        }
        //Read entries.
        ret = com_read(connection, (gxObject*)pg, 8);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            if (connection->trace > GX_TRACE_LEVEL_OFF)
            {
                printf("Failed to read object %s %s attribute index %d\r\n", str, ln, 8);
            }
            //Do not clear objects list because it will free also objects from association view list.
            oa_empty(&objects);
            return ret;
        }
        if (connection->trace > GX_TRACE_LEVEL_WARNING)
        {
            printf("Entries: %ld/%ld\r\n", pg->entriesInUse, pg->profileEntries);
        }
        //If there are no columns or rows.
        if (pg->entriesInUse == 0 || pg->captureObjects.size == 0)
        {
            continue;
        }
        //Read first row from Profile Generic.
        ret = com_readRowsByEntry(connection, pg, 1, 1);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            if (connection->trace > GX_TRACE_LEVEL_OFF)
            {
                printf("Failed to read object %s %s rows by entry\r\n", str, ln);
            }
        }
        else
        {
            if (connection->trace > GX_TRACE_LEVEL_WARNING)
            {
                bb_init(&ba);
                obj_rowsToString(&ba, &pg->buffer);
                data = bb_toString(&ba);
                bb_clear(&ba);
                printf("%s\r\n", data);
                gxfree(data);
            }
        }
        //Read last day from Profile Generic.
        time_now(&startTime);
        endTime = startTime;
        time_clearTime(&startTime);
        struct tm* start;
        struct tm* end;
        start = gmtime((time_t *)&startTime.value);
        end   = gmtime((time_t *)&endTime.value);
        ret   = com_readRowsByRange(connection, pg, start, end);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            if (connection->trace > GX_TRACE_LEVEL_OFF)
            {
                printf("Failed to read object %s %s rows by entry\r\n", str, ln);
            }
        }
        else
        {
            if (connection->trace > GX_TRACE_LEVEL_WARNING)
            {
                bb_init(&ba);
                obj_rowsToString(&ba, &pg->buffer);
                data = bb_toString(&ba);
                bb_clear(&ba);
                printf("%s\r\n", data);
                gxfree(data);
            }
        }
    }
    //Do not clear objects list because it will free also objects from association view list.
    oa_empty(&objects);
    return ret;
}

int com_readValue(connection *connection, gxObject* object, unsigned char index)
{
    int ret;
    char* data = NULL;
    char str[50];
    char ln[25];
    ret = obj_typeToString(object->objectType, str);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    ret = hlp_getLogicalNameToString(object->logicalName, ln);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    if (connection->trace > GX_TRACE_LEVEL_WARNING)
    {
//        printf("-------- Reading Object %s %s\r\n", str, ln);
        LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> -------- Reading Object %s %s\r\n", (int)Tick_Get(SECONDS), str, ln);
    }
    ret = com_read(connection, object, index);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        if (connection->trace > GX_TRACE_LEVEL_OFF)
        {
//            printf("Failed to read object %s %s attribute index %d\r\n", str, ln, index);
        	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Failed to read object %s %s attribute index %d\r\n", (int)Tick_Get(SECONDS), str, ln, index);
        }
        //Return error if not DLMS error.
        if (ret != DLMS_ERROR_CODE_READ_WRITE_DENIED)
        {
            return ret;
        }
    }
    if (connection->trace > GX_TRACE_LEVEL_WARNING)
    {
        ret = obj_toString(object, &data);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            return ret;
        }
        if (data != NULL)
        {
//            printf("%s", data);
            LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> obj_toString %s\r\n", (int)Tick_Get(SECONDS), data);
            gxfree(data);
            data = NULL;
        }
    }
    return 0;
}

// This function reads ALL objects that meter have excluded profile generic objects.
// It will loop all object's attributes.
int com_readValues(connection *connection)
{
    gxByteBuffer attributes;
    unsigned char ch;
    char* data = NULL;
    char str[50];
    char ln[25];
    gxObject* object;
    unsigned long index;
    int ret, pos;
    bb_init(&attributes);

    for (pos = 0; pos != connection->settings.objects.size; ++pos)
    {
        ret = oa_getByIndex(&connection->settings.objects, pos, &object);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            bb_clear(&attributes);
            return ret;
        }
        ///////////////////////////////////////////////////////////////////////////////////
        // Profile generics are read later because they are special cases.
        // (There might be so lots of data and we so not want waste time to read all the data.)
        if (object->objectType == DLMS_OBJECT_TYPE_PROFILE_GENERIC)
        {
            continue;
        }
        ret = obj_typeToString(object->objectType, str);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            bb_clear(&attributes);
            return ret;
        }
        ret = hlp_getLogicalNameToString(object->logicalName, ln);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            bb_clear(&attributes);
            return ret;
        }
        if (connection->trace > GX_TRACE_LEVEL_WARNING)
        {
            printf("-------- Reading Object %s %s\r\n", str, ln);
        }
        ret = obj_getAttributeIndexToRead(object, &attributes);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            bb_clear(&attributes);
            return ret;
        }
        for (index = 0; index < attributes.size; ++index)
        {
            ret = bb_getUInt8ByIndex(&attributes, index, &ch);
            if (ret != DLMS_ERROR_CODE_OK)
            {
                bb_clear(&attributes);
                return ret;
            }
            ret = com_read(connection, object, ch);
            if (ret != DLMS_ERROR_CODE_OK)
            {
                if (connection->trace > GX_TRACE_LEVEL_OFF)
                {
                    printf("Failed to read object %s %s attribute index %d\r\n", str, ln, ch);
                }
                //Return error if not DLMS error.
                if (ret != DLMS_ERROR_CODE_READ_WRITE_DENIED)
                {
                    bb_clear(&attributes);
                    return ret;
                }
                ret = 0;
            }
                }
        bb_clear(&attributes);
        if (connection->trace > GX_TRACE_LEVEL_WARNING)
        {
            ret = obj_toString(object, &data);
            if (ret != DLMS_ERROR_CODE_OK)
            {
                bb_clear(&attributes);
                return ret;
            }
            if (data != NULL)
            {
                printf("%s", data);
                gxfree(data);
                data = NULL;
            }
        }
    }
    bb_clear(&attributes);
    return ret;
}

//This function reads ALL objects that meter have. It will loop all object's attributes.
int com_readAllObjects(connection *connection)
{
    int ret;

    //Initialize connection.
    ret = com_initializeConnection(connection);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    //Get objects from the meter and read them.
    ret = com_getAssociationView(connection);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    ret = com_readScalerAndUnits(connection);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    ///////////////////////////////////////////////////////////////////////////////////
    //Read Profile Generic columns.
    ret = com_readProfileGenericColumns(connection);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    ret = com_readValues(connection);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    ret = com_readProfileGenerics(connection);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    return ret;
}
