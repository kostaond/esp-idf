/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "ksz8863_ctrl.h"
#include "ksz8863.h" //for ksz8863_indir_access_tbls_t

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ksz8863_indirect_read(ksz8863_indir_access_tbls_t tbl, uint8_t ind_addr, void *data, size_t len);
esp_err_t ksz8863_indirect_write(ksz8863_indir_access_tbls_t tbl, uint8_t ind_addr, void *data, size_t len);

#ifdef __cplusplus
}
#endif
