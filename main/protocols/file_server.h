/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* HTTP File Server Example, common declarations

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "esp_http_server.h"


class HttpFileServer {
public:
   HttpFileServer(const char* base_path) {
      start_file_server(base_path);
   }
   ~HttpFileServer() {
      httpd_stop(server);
   }
   httpd_handle_t server = NULL;
   esp_err_t start_file_server(const char *base_path);
   esp_err_t delete_post_handler(httpd_req_t *req);
   esp_err_t upload_post_handler(httpd_req_t *req);
   esp_err_t download_get_handler(httpd_req_t *req);
   const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize);
   esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename);
   esp_err_t http_resp_dir_html(httpd_req_t *req, const char *dirpath);
   esp_err_t favicon_get_handler(httpd_req_t *req);
   esp_err_t index_html_get_handler(httpd_req_t *req);
};
