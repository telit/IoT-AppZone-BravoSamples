/*Copyright (C) 2021 Telit Communications S.p.A. Italy - All Rights Reserved.*/
/*    See LICENSE file in the project root for full license information.     */

#ifndef HDR_APP_CFG_H_
#define HDR_APP_CFG_H_
/**
 * @file app_cfg.h
 * @version 1.0.0
 * @date 27/05/2021
 *
 * @brief Application configuration settings conveniently located here.
 *
 */

/** @cond DEV*/
#define QUOTE(str) #str
#define EXPAND_AND_QUOTE(str) QUOTE(str)
/** @endcond*/

/*Local basepath for samples that need local files usage*/
#define LOCALPATH "/mod"

#endif /* HDR_APP_CFG_H_ */