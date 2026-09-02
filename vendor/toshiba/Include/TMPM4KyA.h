/**
 *******************************************************************************
 * @file    TMPM4KyA.h
 * @brief   CMSIS Cortex-M4 Core Peripheral Access Layer Header File for the
 *          TOSHIBA 'TMPM4Ky' Group.\n
 * @version V1.0.0
 * 
 * DO NOT USE THIS SOFTWARE WITHOUT THE SOFTWARE LICENSE AGREEMENT.
 * 
 * Copyright(C) Toshiba Electronic Device Solutions Corporation 2022
 *******************************************************************************
 */
#ifndef __TMPM4Ky_H__
#define __TMPM4Ky_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup TOSHIBA_TXZp_MICROCONTROLLER TOSHIBA TXZp MICROCONTROLLER
  * @{
  */
  
/** @addtogroup TMPM4Ky TMPM4Ky
  * @{
  */

#if !defined(TMPM4KLA) && !defined(TMPM4KMA) && !defined(TMPM4KNA)
/**
  * @brief Remove a comment of target device.
  */
    /* #define TMPM4KLA */  /*!< TMPM4KLA device */
    /* #define TMPM4KMA */  /*!< TMPM4KMA device */
     #define TMPM4KNA   /*!< TMPM4KNA device */
#endif

/** @addtogroup Device_Included Device Included
  * @{
  */

#if defined(TMPM4KLA)
    #include "TMPM4KLA.h"
#elif defined(TMPM4KMA)
    #include "TMPM4KMA.h"
#elif defined(TMPM4KNA)
    #include "TMPM4KNA.h"
#else
    #error "target device is non-select."
#endif

/**
  * @}
  */ /* End of group Device_Included */

#ifdef __cplusplus
}
#endif

#endif  /* __TMPM4Ky_H__ */

/**
  * @}
  */ /* End of group TMPM4Ky TMPM4Ky */

/**
  * @}
  */ /* End of group TOSHIBA_TXZp_MICROCONTROLLER */
