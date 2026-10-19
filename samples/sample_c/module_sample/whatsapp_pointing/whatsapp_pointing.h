/**
 ********************************************************************
 * @file    whatsapp_pointing.h
 * @brief   Send the aircraft pointing location to a WhatsApp group.
 *
 * Button widget index 0 is named "C3 WhatsApp" and is exposed to the
 * remote controller custom-button list. Map that widget to the C3 key
 * in DJI Pilot: Settings, Remote Controller, Customizable Buttons.
 *
 * @copyright (c) 2026 DJI. All rights reserved.
 *********************************************************************
 */

#ifndef WHATSAPP_POINTING_H
#define WHATSAPP_POINTING_H

#include <dji_typedef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Main-interface button. Pilot maps this widget onto the physical C3 key. */
#define WHATSAPP_POINTING_RC_BUTTON_INDEX    0

/**
 * @brief Read the camera laser pointing location and post it to the
 *        configured WhatsApp group. Returns immediately; the network
 *        request runs on a background thread.
 */
T_DjiReturnCode DjiUser_SendPointingLocationToWhatsApp(void);

#ifdef __cplusplus
}
#endif

#endif /* WHATSAPP_POINTING_H */
