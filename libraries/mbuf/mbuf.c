/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    multi_buffering.c
 * @date    02 Dec 2022
 * @brief   Source file to store data in mbuf
 *
 */

#include "mbuf.h"
#include "coines.h"

static mbuf_t mbuf = { 0 };

static volatile uint8_t buffer_in_use = 0;
static volatile uint8_t curr_buffer_idx = 0;
static volatile bool buffer_in_progress[MBUF_DEPTH] = { false };

/*!
 * @brief This API is used to switch to next buffer.
 *
 */
static void switch_buffer(void)
{
    buffer_in_use = (buffer_in_use + 1) % MBUF_DEPTH;
}

/*!
 * @brief This API is used to clear the buffer.
 *
 */
static void clear_buffer()
{
    memset(mbuf.data[curr_buffer_idx], 0, mbuf.idx[curr_buffer_idx]);
    mbuf.idx[curr_buffer_idx] = 0;
    buffer_in_progress[curr_buffer_idx] = false;
}

/*!
 * @brief This API is used to trigger the buffer clean.
 *
 */
void mbuf_update_processed_buffer()
{
    coines_execute_critical_region(clear_buffer);
}

/*!
 * @brief This API is used to initialize the mbuf.
 *
 */
int8_t mbuf_init(mbuf_evt_handler_t event_handler)
{
    if (mbuf.state == MBUF_STATE_UNINITIALIZED)
    {
        for (uint8_t i = 0; i < MBUF_DEPTH; i++)
        {
            memset(mbuf.data[i], 0, MBUF_DATA_SIZE);
            mbuf.idx[i] = 0;
        }

        mbuf.handler = event_handler;
        mbuf.state = MBUF_STATE_INITIALIZED;
    }

    return MBUF_SUCCESS;

}

/*!
 * @brief This API is used to add data to the mbuf.
 *
 */
void mbuf_add_to_buffer(uint8_t *p_data, uint16_t length)
{
    if (mbuf.state != MBUF_STATE_UNINITIALIZED)
    {
        if ((mbuf.idx[buffer_in_use] + length) >= MBUF_DATA_SIZE)
        {
            /* Update the buffer status */
            buffer_in_progress[buffer_in_use] = true;

            /* Notify the application, Application needs to handle this condition */
            mbuf.handler(MBUF_EVT_BUFFER_FULL);

            /* Save the current index */
            curr_buffer_idx = buffer_in_use;

            /* Switch to next buffer */
            coines_execute_critical_region(switch_buffer);

        }

        memcpy(&mbuf.data[buffer_in_use][mbuf.idx[buffer_in_use]], p_data, length);
        mbuf.idx[buffer_in_use] += length;
    }
}

/*!
 * @brief This API is used to get the data from the mbuf.
 *
 */
int8_t mbuf_get_from_buffer(uint8_t **p_address, uint16_t *p_length)
{
    if (mbuf.state != MBUF_STATE_UNINITIALIZED)
    {
        if (mbuf.idx[buffer_in_use] > 0)
        {
            /* Save the current index */
            curr_buffer_idx = buffer_in_use;

            /* Store the active buffer address */
            *p_address = mbuf.data[buffer_in_use];

            /* Store the active buffer length */
            *p_length = mbuf.idx[buffer_in_use];

            /* Switch to next buffer */
            coines_execute_critical_region(switch_buffer);
        }
        else
        {
            return MBUF_E_READ_FAIL;
        }

        return MBUF_SUCCESS;
    }
    else
    {
        return MBUF_E_INVALID_STATE;
    }
}

/*!
 * @brief This API is used to get the active buffer length.
 *
 */
uint16_t mbuf_get_buffer_length()
{
    return mbuf.idx[buffer_in_use];
}

/*!
 * @brief This API is used to deinit the mbuf.
 *
 */
void mbuf_deinit(void)
{
    if (mbuf.state == MBUF_STATE_INITIALIZED)
    {
        buffer_in_use = 0;
        curr_buffer_idx = 0;
        for (uint8_t i = 0; i < MBUF_DEPTH; i++)
        {
            buffer_in_progress[i] = false; 
        }
        mbuf.handler = NULL;
        mbuf.state = MBUF_STATE_UNINITIALIZED;
    }   
}
