/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"

#define RAM_FLASHLOADER_ADDR    0x0      /* itcm base */

/*******************************************************************************
 * Variables
 ******************************************************************************/

typedef void     (p_flexspi_nor_init_t) (void);
typedef status_t (p_get_vendor_id_t) (FLEXSPI_Type *base, uint8_t *vendorId);
typedef status_t (p_write_enable_t) (FLEXSPI_Type *base, uint32_t baseAddr);
typedef status_t (p_wait_bus_busy_t) (FLEXSPI_Type *base);    
typedef status_t (p_erase_sector_t) (FLEXSPI_Type *base, uint32_t address);
typedef status_t (p_page_program_t) (FLEXSPI_Type *base, uint32_t dstAddr, const uint32_t *src, uint32_t len);
typedef status_t (p_enable_quad_mode_t) (FLEXSPI_Type *base);


typedef struct __ram_flashloader_info {
    uint32_t tag;               // == "RFLI" == 0x52464C49;
    uint32_t version;           //[31:16] major [15:0] minor
    p_flexspi_nor_init_t    *p_flexspi_init;
    p_get_vendor_id_t       *p_get_vendor_id;
    p_write_enable_t        *p_write_enable;
    p_wait_bus_busy_t       *p_wait_bus_busy;
    p_erase_sector_t        *p_erase_sector;
    p_page_program_t        *p_page_program;
    p_enable_quad_mode_t    *p_enable_quad_mode;

} ram_flashloader_info_t;


extern ram_flashloader_info_t ram_flashloader;

/*******************************************************************************
 * Code
 ******************************************************************************/










