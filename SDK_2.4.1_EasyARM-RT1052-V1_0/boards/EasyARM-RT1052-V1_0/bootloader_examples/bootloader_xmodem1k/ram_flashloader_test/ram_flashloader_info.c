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

#include "ram_flashloader_info.h"



/*******************************************************************************
 * Variables
 ******************************************************************************/

#if  0  // O1

ram_flashloader_info_t ram_flashloader = {
    .tag = 0x52464C49,              //"RFLI"
    .version = 0x00009000,          //"V0.90"

    .p_flexspi_init     = (p_flexspi_nor_init_t *)  (RAM_FLASHLOADER_ADDR + 0x0000139f),
    .p_get_vendor_id    = (p_get_vendor_id_t *)     (RAM_FLASHLOADER_ADDR + 0x00001361),
    .p_write_enable     = (p_write_enable_t *)      (RAM_FLASHLOADER_ADDR + 0x000013f1),
    .p_wait_bus_busy    = (p_wait_bus_busy_t *)     (RAM_FLASHLOADER_ADDR + 0x000013a1),
    .p_erase_sector     = (p_erase_sector_t *)      (RAM_FLASHLOADER_ADDR + 0x0000128f),
    .p_page_program     = (p_page_program_t *)      (RAM_FLASHLOADER_ADDR + 0x000012f7),
    .p_enable_quad_mode = (p_enable_quad_mode_t *)  (RAM_FLASHLOADER_ADDR + 0x00001231) 

};

#else 

// O3
ram_flashloader_info_t ram_flashloader = {
    .tag = 0x52464C49,              //"RFLI"
    .version = 0x00009000,          //"V0.90"

    .p_flexspi_init     = (p_flexspi_nor_init_t *)  (RAM_FLASHLOADER_ADDR + 0x00000e55),
    .p_get_vendor_id    = (p_get_vendor_id_t *)     (RAM_FLASHLOADER_ADDR + 0x00000e21),
    .p_write_enable     = (p_write_enable_t *)      (RAM_FLASHLOADER_ADDR + 0x00000e91),
    .p_wait_bus_busy    = (p_wait_bus_busy_t *)     (RAM_FLASHLOADER_ADDR + 0x00000e57),
    .p_erase_sector     = (p_erase_sector_t *)      (RAM_FLASHLOADER_ADDR + 0x00000d6d),
    .p_page_program     = (p_page_program_t *)      (RAM_FLASHLOADER_ADDR + 0x00000dc9),
    .p_enable_quad_mode = (p_enable_quad_mode_t *)  (RAM_FLASHLOADER_ADDR + 0x00000d1f) 

};

#endif
/*******************************************************************************
 * Code
 ******************************************************************************/










