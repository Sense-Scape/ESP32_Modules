#include "WifiModule.h"

// static const char *payload = "Message from ESP32 ";

WifiModule::WifiModule(std::string sSSID, std::string sPassword, std::string sHostIPAddress, unsigned uUDPport, unsigned uDatagramSize,unsigned uBufferSize) :
BaseModule(uBufferSize),
m_sSSID(sSSID),
m_sPassword(sPassword),
m_sHostIPAddress(sHostIPAddress),
m_uUDPPort(uUDPport),
m_uDatagramSize(uDatagramSize),
m_uSessionNumber(0)
{
    ConnectWifiConnection();
    ConnectToSocket();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void WifiModule::Process(std::shared_ptr<BaseChunk> pBaseChunk)
{
    SendUDP(pBaseChunk);
}

void WifiModule::ConnectWifiConnection()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1 - Wi-Fi/LwIP Init Phase
    ESP_ERROR_CHECK(esp_netif_init());                // TCP/IP initiation  s1.1
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // event loop 		s1.2
    esp_netif_create_default_wifi_sta();              // WiFi station       s1.3
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_initiation)); //                    s1.4

    // 2 - Wi-Fi Configuration Phase
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_configuration = {.sta = {
                                            /* ssid            */ {},
                                            /* password        */ {},
                                            /* scan_method     */ {},
                                            /* bssid_set       */ {},
                                            /* bssid           */ {},
                                            /* channel         */ {},
                                            /* listen_interval */ {},
                                            /* sort_method     */ {},
                                            /* threshold       */ {/* rssi            */ {},
                                                                   /* authmode        */ WIFI_AUTH_WPA2_PSK},
                                            /* pmf_cfg         */ {/* capable         */ true,
                                                                   /* required        */ false},
                                            /* rm_enabled      */ {},
                                            /* btm_enabled     */ {},
                                            // /* mbo_enabled     */ {}, // For IDF 4.4 and higher
                                            /* reserved        */ {}}};

    memcpy(&wifi_configuration.sta.ssid[0], &m_sSSID[0], sizeof(m_sSSID));
    memcpy(&wifi_configuration.sta.password[0], &m_sPassword[0], sizeof(m_sPassword));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(m_TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(m_s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
        ESP_LOGI(m_TAG, "connected to ap SSID:%s password:%s", m_sSSID.c_str(), m_sPassword.c_str());
    else if (bits & WIFI_FAIL_BIT)
        ESP_LOGI(m_TAG, "Failed to connect to SSID:%s, password:%s", m_sSSID.c_str(), m_sPassword.c_str());
    else
        ESP_LOGE(m_TAG, "UNEXPECTED EVENT");
}

void WifiModule::ConnectToSocket()
{
    m_dest_addr.sin_addr.s_addr = inet_addr(m_sHostIPAddress.c_str());
    m_dest_addr.sin_family = AF_INET;
    m_dest_addr.sin_port = htons(m_uUDPPort);

    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    m_sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
}

void WifiModule::wifi_event_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < 3)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(m_TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(m_s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(m_TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(m_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(m_s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void WifiModule::SendUDP(std::shared_ptr<BaseChunk> pBaseChunk)
{
    // Serialising chunk to bytes
    auto pvcByteData = pBaseChunk->Serialise();
    
    // Bytes to transmit is equal to number of bytes in derived object (e.g TimeChunk)
    uint32_t dTransmittableBytes = pvcByteData->size();

    // Transmission state vartiables - This is essentially TCP now
    unsigned uSequenceNumber = 0; // sequence 0 indicated error, 1 is starting
    unsigned uDatagramHeaderSize = 24; // NEEDS TO RESULT IN DATA WITH MULTIPLE OF 4 BYTES
    uint8_t uTransmissionState = 0;    // 0 - error 1 - active transmission 2 - complete
    unsigned uDataBytesTransmitted = 0;
    unsigned uSessionNumber = m_uSessionNumber;
    unsigned uTransmissionSize = m_uDatagramSize;
    unsigned uMaxTranssionSize = m_uDatagramSize; // bytes
    uint32_t u32ChunkType = ChunkTypesUtility::toU32(pBaseChunk->GetChunkType());
    bool bTransmit = true;

    // Logic for transimission
    while (bTransmit)
    {
        // If not enough bytes for a full transmission
        if (uDataBytesTransmitted + uTransmissionSize - uDatagramHeaderSize> dTransmittableBytes)
        {
            uTransmissionSize = dTransmittableBytes - uDataBytesTransmitted + uDatagramHeaderSize;
            uTransmissionState = 1;
            bTransmit = false;
        }
        else
        {
            uTransmissionSize = uMaxTranssionSize;
        }

        // UDP transmission state control
        uint8_t auUDPData[uTransmissionSize] = {(uint8_t)0};
        memcpy(&auUDPData[0], &uSequenceNumber, sizeof(uSequenceNumber)); // 4 bytes
        memcpy(&auUDPData[4], &uTransmissionState, sizeof(uTransmissionState));
        memcpy(&auUDPData[5], &uTransmissionSize, sizeof(uTransmissionSize));
        memcpy(&auUDPData[9], &u32ChunkType, sizeof(u32ChunkType));
        memcpy(&auUDPData[13], &uSessionNumber, sizeof(uSessionNumber));

        //TODO: implement IP tx to uniquley identify device

        // Actual Byte data to transmit
        memcpy(&auUDPData[uDatagramHeaderSize], &((*pvcByteData)[uDataBytesTransmitted]), uTransmissionSize - uDatagramHeaderSize);

        const void *ptr_payload(&(auUDPData));
        int err = sendto(m_sock, ptr_payload, uTransmissionSize, 0, (struct sockaddr *)&m_dest_addr, sizeof(m_dest_addr));

        // Only print if there was a TX error
        if (err < 0)
            ESP_LOGE(m_TAG, "Error occurred during sending: errno %d ", errno);

        // Updating transmission states
        uDataBytesTransmitted = uDataBytesTransmitted + uTransmissionSize - uDatagramHeaderSize;
        uSequenceNumber++;
        
    }

    m_uSessionNumber++;
}