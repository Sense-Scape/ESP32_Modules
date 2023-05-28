#include "WifiModule.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

WifiModule::WifiModule(std::string sSSID, std::string sPassword, std::string sHostIPAddress, std::string strMode, unsigned uPort, unsigned uDatagramSize, unsigned uBufferSize) : BaseModule(uBufferSize),
                                                                                                                                                                                  m_sSSID(sSSID),
                                                                                                                                                                                  m_sPassword(sPassword),
                                                                                                                                                                                  m_sHostIPAddress(sHostIPAddress),
                                                                                                                                                                                  m_strMode(strMode),
                                                                                                                                                                                  m_uPort(uPort),
                                                                                                                                                                                  m_uDatagramSize(uDatagramSize),
                                                                                                                                                                                  m_uSessionNumber(0)
{
    // Connect to the WiFi
    ConnectWifiConnection();
    // And then create either a UDP or TCP connection
    ConnectToSocket();
    // Then Give some time for everything to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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

void WifiModule::ConnectTCPSocket()
{
    m_dest_addr.sin_addr.s_addr = inet_addr(m_sHostIPAddress.c_str());
    m_dest_addr.sin_family = AF_INET;
    m_dest_addr.sin_port = htons(m_uPort);
    m_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    
    // Internet address presentation to network address conversion - IPV4 v.s IPV6
    inet_pton(AF_INET, m_sHostIPAddress.c_str(), &m_dest_addr.sin_addr.s_addr);

    int optval = 1;
    int optlen = sizeof(optval);
    if (setsockopt(m_sock, SOL_SOCKET, SO_KEEPALIVE, (char *)&optval, optlen) < 0)
        return;

     // Disable Nagle's algorithm (TCP_NODELAY)
    int flag = 1;
    if (setsockopt(m_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int)) == -1) {
        std::cerr << "Failed to set TCP_NODELAY option." << std::endl;
        return;
    }

    if (connect(m_sock, (struct sockaddr *)&m_dest_addr, sizeof(m_dest_addr)) < 0)
    {
        ESP_LOGE(__func__, "Failed to connect to server: errno %d", errno);
        close(m_sock);
        return;
    }
    else
        std::cout << std::string(__PRETTY_FUNCTION__) + ": Connected to TCP server on " + m_sHostIPAddress + ":" + std::to_string(m_uPort) + "\n";
}

void WifiModule::ConnectUDPSocket()
{
    // Lets start by setting up transmission protocol information
    m_dest_addr.sin_addr.s_addr = inet_addr(m_sHostIPAddress.c_str());
    m_dest_addr.sin_family = AF_INET;
    m_dest_addr.sin_port = htons(m_uPort);

    // And then create hte socket
    m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    std::cout << std::string(__PRETTY_FUNCTION__) + ": WiFi mode UDP\n";
}

void WifiModule::ConnectToSocket()
{
    // Lets first check what mode we are transmitting in
    if (m_strMode == "UDP")
        // and connect using UDP
        ConnectUDPSocket();
    else if (m_strMode == "TCP")
        // Or TCP
        ConnectTCPSocket();
    else
        // But lets default to UDP, just in case
        ConnectUDPSocket();
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
    // Bytes to transmit is equal to number of bytes in derived object (e.g TimeChunk)
    auto pvcByteData = pBaseChunk->Serialise();
    uint32_t u32TransmittableDataBytes = pvcByteData->size();
    uint32_t u32ChunkType = ChunkTypesUtility::ToU32(pBaseChunk->GetChunkType());
 
    // Intra-transmission state information
    unsigned uDatagramHeaderSize = 24;      // NEEDS TO RESULT IN DATA WITH MULTIPLE OF 4 BYTES
    uint16_t uSessionDataHeaderSize = 2;          // size of the footer in bytes. '\0' denotes finish if this structure
    uint16_t uSessionTransmissionSize = m_uDatagramSize;
    uint8_t uTransmissionState = 0;         // 0 - Transmitting; 1 - finished

    // Inter-transmission state information
    bool bTransmit = true;
    unsigned uSequenceNumber = 0;           // sequence 0 indicated error, 1 is starting
    unsigned uDataBytesTransmitted = 0;     // Current count of how many bytes have been transmitted
    unsigned uDataBytesToTransmit = m_uDatagramSize - uSessionDataHeaderSize - uDatagramHeaderSize;
    unsigned uMaxTransmissionSize = m_uDatagramSize; // Largest buffer size that can be request for transmission
    uint32_t uSessionNumber = m_uSessionNumber;

    // Now that we have configured meta data, lets start transmitting
    while (bTransmit)
    {
        // If our next transmission exceeds the number of data bytes available to be transmitted
        if (uDataBytesTransmitted + uDataBytesToTransmit > u32TransmittableDataBytes)
        {
            // Then adjust to how many data bytes shall be transmitted to the remaining number
            uDataBytesToTransmit = u32TransmittableDataBytes - uDataBytesTransmitted;
            uSessionTransmissionSize = uDataBytesToTransmit +  uDatagramHeaderSize + uSessionDataHeaderSize;
            // And then inform process to finish up
            uTransmissionState = 1;
            bTransmit = false;
        }

        // Lets first insert the header transmission state information into the bytes array
        uint8_t auUDPData[uMaxTransmissionSize] = {(uint8_t)0};
        memcpy(&auUDPData[0], &uSessionTransmissionSize, uSessionDataHeaderSize); 
        memcpy(&auUDPData[0+uSessionDataHeaderSize], &uSequenceNumber, sizeof(uSequenceNumber)); // 4 bytes
        memcpy(&auUDPData[4+uSessionDataHeaderSize], &uTransmissionState, sizeof(uTransmissionState));
        memcpy(&auUDPData[5+uSessionDataHeaderSize], &uDataBytesToTransmit, sizeof(uDataBytesToTransmit));
        memcpy(&auUDPData[9+uSessionDataHeaderSize], &u32ChunkType, sizeof(u32ChunkType));
        memcpy(&auUDPData[13+uSessionDataHeaderSize], &uSessionNumber, sizeof(uSessionNumber));

        // TODO: implement IP tx to uniquely identify device

        // Then lets insert the actual data byte data to transmit after the header
        // While keeping in mind that we have to send unset bits from out data byte array
        // { | DataHeaderSize | DatagramHeader | Data | }
        memcpy(&auUDPData[uSessionDataHeaderSize + uDatagramHeaderSize], &((*pvcByteData)[uDataBytesTransmitted]), uDataBytesToTransmit);

        // Then lets insert footer information after header and data
        // so the receiver knows when this message is complete;
        const void *ptr_payload(&(auUDPData));

        int err;
        if (m_strMode == "UDP")
            do
            {
                err = sendto(m_sock, ptr_payload, uSessionTransmissionSize, 0, (struct sockaddr *)&m_dest_addr, sizeof(m_dest_addr));
                if (err != uSessionTransmissionSize)
                {
                    ESP_LOGE(m_TAG, "Error occurred during sending: errno %d - Retrying", errno);
                }

            } while (err != uSessionTransmissionSize); // if udp transmit did not end up in a memory issue
        else
            // Sen using TCP 
            err = send(m_sock, ptr_payload, uSessionTransmissionSize, 0);

        //  Only print if there was a TX error
        if (err == 0)
        {
            // We have disconnected so lets try reconnect
            ESP_LOGE(m_TAG, "Error occurred during sending: errno %d - Socket disconnect", errno);
            ConnectToSocket();
        }
        else if (err < 0)
        {
            // Otherwise it was some other error
            ESP_LOGE(m_TAG, "Error occurred during sending: errno %d ", errno);
            if (errno == 118)
                esp_restart(); // TODO: Complete proper handling of incorrect TX
        }

        // Updating transmission states
        uDataBytesTransmitted+=uDataBytesToTransmit;
        uSequenceNumber++;
    }
    m_uSessionNumber++;
}
