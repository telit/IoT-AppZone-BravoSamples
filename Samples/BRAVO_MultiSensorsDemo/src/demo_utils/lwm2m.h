/*===============================================================================================*/
/*         >>> Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved. <<<          */
/**
  @file
    lwm2m.h

  @brief
    demo lwm2m utilities

  @details


  @note
    Dependencies:
    m2mb_types.h
    m2mb_lwm2m.h

  @author WhiteBeard
  @author FabioPi

  @date
    2020-02-15
*/
/* Global declarations ==========================================================================*/
/* Global typedefs ==============================================================================*/


#ifndef SRC_LWM2M_H_
#define SRC_LWM2M_H_

/* Global defines ================================================================================*/
/*OBJECTs and RESOURCEs IDs*/
#define LWM2M_LIGHT_CONTROL_OBJ_ID	3311
#define LWM2M_LIGHT_OBJECT_XML_NAME "object_3311.xml"

#define LIGHT_CONTROL_OBJ_ON_OFF_ID 5850
#define LIGHT_CONTROL_OBJ_DIMMER_ID 5851
#define LIGHT_CONTROL_OBJ_TIME_ID 5852
#define LIGHT_CONTROL_OBJ_CUMULATIVE_ACTIVE_POWER_ID 5805
#define LIGHT_CONTROL_OBJ_POWER_FACTOR_ID 5820
#define LIGHT_CONTROL_OBJ_COLOR_ID 5806
#define LIGHT_CONTROL_OBJ_SENSOR_UNITSR_ID 5701
#define LIGHT_CONTROL_OBJ_APPLICATION_TYPE_ID 5750


#define TAMPERING_OBJ_ID 26242
#define ENVIRONMENT_OBJ_ID 26251
#define ROTATION_OBJ_ID 26250

#define TIME_SERIES_METERING_INFO_OBJ_ID  32001
#define TIME_SERIES_METERING_RATES_OBJ_ID 32002

#define TELIT_APPLICATION_DATA_OBJ_ID 33205


#define EV_LWM2M_ENABLE_RES_BIT         (UINT32)0x00000001
#define EV_LWM2M_DISABLE_RES_BIT        (UINT32)0x00000002
#define EV_LWM2M_SET_RES_BIT            (UINT32)0x00000004
#define EV_LWM2M_NEW_RES_BIT            (UINT32)0x00000008
#define EV_LWM2M_GET_RES_BIT            (UINT32)0x00000010
#define EV_LWM2M_MON_RES_BIT            (UINT32)0x00000020
#define EV_LWM2M_WRITE_RES_BIT          (UINT32)0x00000040
#define EV_LWM2M_READ_RES_BIT           (UINT32)0x00000080
#define EV_LWM2M_SRV_REG_BIT            (UINT32)0x00000100
#define EV_LWM2M_GET_STAT_RES_BIT       (UINT32)0x00000200
#define EV_LWM2M_GET_LIST_RES_BIT       (UINT32)0x00000400
#define EV_LWM2M_SRV_DISABLED_BIT       (UINT32)0x00000800
#define EV_LWM2M_NFYADD_RES_BIT         (UINT32)0x00002000
#define EV_LWM2M_NFYACK_BIT             (UINT32)0x00004000
#define EV_LWM2M_NFYACK_LIST_RES_BIT    (UINT32)0x00008000
#define EV_LWM2M_NFYSTAT_RES_BIT        (UINT32)0x00010000

#define EV_MON_URC_RECEIVED  	 2
#define EV_LWM2M_WRITE_APP_TYPE  3
#define CTX_ID 1 //default cid used by lwm2m

#define LIGHT_CONTROL_OBJ_INFO(desc, ap, i, fdescr, p) 	\
                      { .description = (char*)(desc), 	\
						.appType = (char*)(ap), 		\
						.index = (i),				    \
						.fd = (fdescr),  			    \
						.pin = (p),  			 	    \
                      }

/* Global typedefs ===============================================================================*/

typedef struct LIGHT_CONTROL_OBJ_INSTANCE_TAG
{
  char 	*description;
  char 	*appType;
  int 	index;
  int 	fd;
  int 	pin;
} LIGHT_CONTROL_OBJ_INSTANCE_S;

typedef struct
{
  INT32 obj_id;
  INT16 inst_num;
  INT16 *instances_ids;
} LWM2M_OBJ_REG_T;

typedef struct
{
  void *data;
  INT32 objID;
} LWM2M_OBJ_USERDATA_T;

/* Global statics ================================================================================*/


/* Global functions prototype =============================================================================*/

/**
  @brief        Callback function for Client generated LWM2M events

  @param[in]    h           LWM2M handle
  @param[in]    event       Event Identification
  @param[in]    resp_size   Size of the response buffer
  @param[in]    resp_struct Address of response buffer
  @param[in]    userdata    Address of user data

*/
void lwm2mIndCB( M2MB_LWM2M_HANDLE h, M2MB_LWM2M_EVENT_E event, UINT16 resp_size, void *resp_struct,
                 void *userdata );


/**
  @brief        If called, the oneedge_init will skip the new instances registration  for provided objects
  @return       none
*/
void setAsBootstrapped(void);


/**
  @brief        If called, the oneedge_init will perform the new instances registration for provided objects
  @return       none
*/
void setAsNotBootstrapped(void);

/**
  @brief        Initialize OneEdge connection
  @param[in]    obj_id      Object ID to be used for initialization
  @return       result of initialization

*/
uint8_t oneedge_init( LWM2M_OBJ_REG_T *objs, INT16 obj_num, void *userData );

/**
  @brief        Write a value to LWM2M Tamper uri

  @param[in]    value    Integer value to be written

*/
void update_tamper_LWM2MObject( int value );



/**
  @brief        Write a value to LWM2M Environmental uri

  @param[in]    _t    Temperature to be written
  @param[in]    _p    Pressure value to be written
  @param[in]    _rh   Humidity value to be written
  @param[in]    _iaq  Index Air Quality value to be written

*/
void update_environment_LWM2MObject( float _t, float _p, float _rh, INT16 _iaq );


/**
  @brief        Write a value to LWM2M Rotation uri

  @param[in]    _w    w parameter
  @param[in]    _x    x component
  @param[in]    _y    y component
  @param[in]    _z    z component
  @param[in]    _acc  accuracy

*/
void update_rotation_LWM2MObject( float _w, float _x, float _y, float _z, INT16 _acc );


/**
  @brief        Check existence of XML object description file in XML directory

  @param[in]    name    Name of the file to be checked
  @retval               0 if OK, other values error

*/
int check_xml_file( const char *name );


/**
  @brief        Copy XML file from /mod to /XML directory

  @param[in]    name    Name of the file to be checked
  @retval               0 if OK, other values error

*/
int copy_xml_file( const char *name );


/**
  @brief        Retrun the LIGHT_CONTROL_OBJ_INSTANCE_S table

  @param[in]    name    Name of the LIGHT_CONTROL_OBJ_INSTANCE_S object
  @retval               1 if OK, -1 values error

*/
int getLightControlObjTable( LIGHT_CONTROL_OBJ_INSTANCE_S **lightObjs );


/**
  @brief        Register the taskID called in to lwm2m_taskCB function

  @param[in]    name    Task ID
  @retval               1 if OK, -1 values error

*/
int lwm2m_ReadTaskRegister( INT32 taskID);

/**
  @brief        Return the lwm2m handle
*/
M2MB_LWM2M_HANDLE getLwM2mHandle();

/**
  @brief        Return the event lwm2m handle
*/
M2MB_OS_EV_HANDLE getEventHandle();


/**
  @brief        Register a monitor on a resource by creating a URI object and passing it to m2mb_lwm2m_mon

  @param[in]    uri
  @param[in]    mon

  @retval               1 if OK, -1 values error

*/
int registerMonitorResource(M2MB_LWM2M_OBJ_URI_T uri_mon, M2MB_LWM2M_MON_REQ_T mon);


/**
  @brief        read an integer value from an uri


  @param[in]    h lwm2m initialized handle ( can be retrieved with getEventHandle() )
  @param[in]    pUri pointer to the URI from which data will be read
  @param[out]    result pointer to the allocated variable that will be filled with the integer value

  @retval       M2MB_RESULT_E value

*/
M2MB_RESULT_E read_integer_from_uri(M2MB_LWM2M_HANDLE h, M2MB_LWM2M_OBJ_URI_T *pUri, INT32 *result);

/**
  @brief        save an integer value to an uri

  @param[in]    h lwm2m initialized handle ( can be retrieved with getEventHandle() )
  @param[in]    pUri pointer to the URI from which data will be read
  @param[in]    integer value to be set

  @retval       0 in case of success, a negative value in case o failure

*/
INT32 write_integer_resource(M2MB_LWM2M_HANDLE h, M2MB_LWM2M_OBJ_URI_T *pUri, INT32 value);

/**
  @brief        set an opaque to an uri

  @param[in]    h lwm2m initialized handle ( can be retrieved with getEventHandle() )
  @param[in]    pUri pointer to the URI from which data will be read
  @param[in]    data pointer to opaque buffer
  @param[in]    datalen size of the opaque buffer

  @retval       0 in case of success, a negative value in case o failure

*/
INT32 set_opaque_resource(M2MB_LWM2M_HANDLE h, M2MB_LWM2M_OBJ_URI_T *pUri, UINT8 *data, UINT32 datalen);

#endif /* SRC_LWM2M_H_ */
