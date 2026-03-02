/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : midi_engine.c
  * @brief          : USB MIDI 1.0 packet parser and note management.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "midi_engine.h"
#include "usbd_composite.h"
#include "scheduler.h"
#include "safety.h"
#include "debug_uart.h"
#include <string.h>

/* ---------------------------------------------------------------------------*/
/*                         Private Variables                                  */
/* ---------------------------------------------------------------------------*/

/** 128-entry MIDI note-to-frequency lookup table (Hz). */
static float g_freq_table[128];

/** Channel-to-coil routing map.  0xFF = unmapped. */
static uint8_t g_channel_to_coil[MIDI_MAX_CHANNELS];

/* ---------------------------------------------------------------------------*/
/*                         Public API                                        */
/* ---------------------------------------------------------------------------*/

void MidiEngine_Init(void)
{
  /*
   * Build the frequency table using iterative multiplication.
   * Equal temperament: freq(n) = 440 * 2^((n-69)/12)
   * We avoid exp2f/powf to eliminate math library dependency.
   *
   * Semitone ratio = 2^(1/12) ≈ 1.05946309436
   */
  g_freq_table[69] = 440.0f;   /* A4 = 440 Hz */

  const float ratio = 1.05946309436f;

  for (int i = 70; i < 128; i++)
    g_freq_table[i] = g_freq_table[i - 1] * ratio;

  for (int i = 68; i >= 0; i--)
    g_freq_table[i] = g_freq_table[i + 1] / ratio;

  /*
   * Default channel-to-coil routing:
   *   CH 0 → Coil 0, CH 1 → Coil 1, ..., CH 5 → Coil 5
   *   CH 6-15 → unmapped
   */
  for (uint8_t i = 0; i < MIDI_MAX_CHANNELS; i++)
  {
    g_channel_to_coil[i] = (i < NUM_COILS) ? i : MIDI_CHANNEL_UNMAP;
  }

  Debug_Log("[MIDI] Engine initialized (128-note freq table built)");
}

void MidiEngine_ProcessUSB(USBD_HandleTypeDef *pdev)
{
  uint8_t packet[4];

  while (USBD_Composite_MIDI_ReadPacket(pdev, packet))
  {
    /*
     * USB MIDI 1.0 Event Packet:
     *   Byte 0: [7:4] Cable Number, [3:0] Code Index Number (CIN)
     *   Byte 1: MIDI status byte
     *   Byte 2: MIDI data byte 1
     *   Byte 3: MIDI data byte 2
     */
    uint8_t cin     = packet[0] & 0x0FU;
    uint8_t channel = packet[1] & 0x0FU;
    uint8_t data1   = packet[2];     /* Note number or controller      */
    uint8_t data2   = packet[3];     /* Velocity or controller value   */

    uint8_t coil_id = g_channel_to_coil[channel];
    if (coil_id >= NUM_COILS) continue;   /* Channel not mapped */

    switch (cin)
    {
      /* ---- Note On (0x9n) ---- */
      case 0x09:
      {
        if (data2 == 0)
        {
          /* Velocity 0 = Note Off */
          Scheduler_RemoveTone(coil_id, data1, channel);
          Debug_Printf("[MIDI] NoteOff ch=%u note=%u coil=%u",
                       channel, data1, coil_id);
        }
        else
        {
          float freq = g_freq_table[data1 & 0x7F];
          uint32_t period_us = (freq > 0.5f)
              ? (uint32_t)(1000000.0f / freq)
              : 65535U;

          /* On-time proportional to velocity:
           *   ontime = velocity/127 * max_ontime_us
           * Uses integer math: (vel * max_ot) / 127  */
          SafetyLimits_t lim;
          Safety_GetLimits(coil_id, &lim);
          uint16_t ontime_us = (uint16_t)((uint32_t)data2 * lim.max_ontime_us / 127U);
          if (ontime_us < 1U) ontime_us = 1U;

          int8_t slot = Scheduler_AddTone(coil_id, data1, channel,
                                           data2, ontime_us, period_us);

          Debug_Printf("[MIDI] NoteOn ch=%u note=%u vel=%u coil=%u per=%lu ot=%u slot=%d",
                       channel, data1, data2, coil_id,
                       (unsigned long)period_us, ontime_us, (int)slot);
        }
        break;
      }

      /* ---- Note Off (0x8n) ---- */
      case 0x08:
      {
        Scheduler_RemoveTone(coil_id, data1, channel);
        Debug_Printf("[MIDI] NoteOff ch=%u note=%u coil=%u",
                     channel, data1, coil_id);
        break;
      }

      /* ---- Control Change (0xBn) ---- */
      case 0x0B:
      {
        /* CC 120 = All Sound Off, CC 123 = All Notes Off */
        if (data1 == 120 || data1 == 123)
        {
          Scheduler_RemoveAllTones(coil_id);
          Debug_Printf("[MIDI] AllNotesOff ch=%u coil=%u", channel, coil_id);
        }
        break;
      }

      default:
        /* Ignore other MIDI messages for now */
        break;
    }
  }
}

void MidiEngine_SetChannelCoil(uint8_t channel, uint8_t coil_id)
{
  if (channel < MIDI_MAX_CHANNELS)
  {
    g_channel_to_coil[channel] = coil_id;
  }
}

uint8_t MidiEngine_GetChannelCoil(uint8_t channel)
{
  if (channel >= MIDI_MAX_CHANNELS) return MIDI_CHANNEL_UNMAP;
  return g_channel_to_coil[channel];
}

float MidiEngine_NoteToFreq(uint8_t note)
{
  if (note > 127) return 0.0f;
  return g_freq_table[note];
}
