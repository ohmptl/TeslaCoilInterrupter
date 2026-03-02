/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : midi_engine.h
  * @brief          : USB MIDI 1.0 packet parser and note management.
  *
  * Processes 4-byte USB MIDI event packets from the composite device's
  * MIDI ring buffer.  Maps MIDI channels to coils and converts Note On/Off
  * events into scheduler tones.
  *
  * Frequency table:
  *   128-entry lookup table built at init using iterative semitone ratio
  *   multiplication (no math library required).  Matches standard equal
  *   temperament: A4 (note 69) = 440 Hz.
  *
  * Channel routing:
  *   Default: MIDI channels 0-5 map to coils 0-5.
  *   Channels 6-15 are unmapped (0xFF).
  *   Re-mappable via CDC command (Milestone 3 full routing matrix).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MIDI_ENGINE_H
#define __MIDI_ENGINE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "usbd_def.h"

/* ---------------------------------------------------------------------------*/
/*                          Constants                                         */
/* ---------------------------------------------------------------------------*/
#define MIDI_MAX_CHANNELS   16U
#define MIDI_CHANNEL_UNMAP  0xFFU   /* Channel not routed to any coil */

/* ---------------------------------------------------------------------------*/
/*                          Public API                                        */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Initialize frequency table and default channel-to-coil routing.
 */
void MidiEngine_Init(void);

/**
 * @brief  Process all pending USB MIDI packets from the composite device.
 *         Handles Note On, Note Off, and All Notes Off (CC 120/123).
 *         Called from main loop.
 */
void MidiEngine_ProcessUSB(USBD_HandleTypeDef *pdev);

/**
 * @brief  Set channel-to-coil routing.
 * @param  channel  MIDI channel 0-15
 * @param  coil_id  Coil index 0-5, or MIDI_CHANNEL_UNMAP to disable.
 */
void MidiEngine_SetChannelCoil(uint8_t channel, uint8_t coil_id);

/**
 * @brief  Get current coil assignment for a MIDI channel.
 */
uint8_t MidiEngine_GetChannelCoil(uint8_t channel);

/**
 * @brief  Convert a MIDI note number (0-127) to frequency in Hz.
 * @return Frequency in Hz (float).  0.0 for invalid note.
 */
float MidiEngine_NoteToFreq(uint8_t note);

#ifdef __cplusplus
}
#endif

#endif /* __MIDI_ENGINE_H */
