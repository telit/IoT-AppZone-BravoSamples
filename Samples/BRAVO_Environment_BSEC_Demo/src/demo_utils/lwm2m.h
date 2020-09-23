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
/* Global functions =============================================================================*/
#ifndef SRC_LWM2M_H_
#define SRC_LWM2M_H_

#define TAMPERING_OBJ_ID 26242
#define ENVIRONMENT_OBJ_ID 26251
#define ROTATION_OBJ_ID 26250
#define SMARTLOCK_OBJ_ID 26247


#define EV_SRV_REG_BIT         (UINT32)0x1    /*0x0000000000000001*/


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
  @brief        Initialize OneEdge connection
  @param[in]    obj_id      Object ID to be used for initialization
  @return       result of initialization

*/
uint8_t oneedge_init( INT32 obj_id );

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
  @brief        Write a value to LWM2M SmartLock uri

  @param[in]    value  integer to be written

*/
void update_smartlock_LWM2MObject( int value );

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


#endif /* SRC_LWM2M_H_ */
