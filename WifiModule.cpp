#include "WifiModule.h"

// static const char *payload = "Message from ESP32 ";

WifiModule::WifiModule(std::string sSSID, std::string sPassword, std::string sHostIPAddress, unsigned uUDPport, unsigned uBufferSize) :
BaseModule(uBufferSize),
m_sSSID(sSSID),
m_sPassword(sPassword),
m_sHostIPAddress(sHostIPAddress),
m_uUDPPort(uUDPport)
{
    ConnectWifiConnection();
    ConnectToSocket();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void WifiModule::Process(std::shared_ptr<BaseChunk> pBaseChunk)
{
    std::shared_ptr<TimeChunk> pTimeChunk = std::static_pointer_cast<TimeChunk>(pBaseChunk);
    // Converting to WAV then transmitting via UDP
    SendUDP(std::make_shared<WAVFile>(ConvertTimeChunkToWAV(pTimeChunk)));
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

void WifiModule::SendUDP(std::shared_ptr<WAVEFile> psWAVFile)
{
    
    // Bytes to transmit is equal to number of bytes in WAV file
    uint32_t dTransmittableBytes = psWAVFile->sWavHeader.ChunkSize + 8 - 44;
    unsigned uSequenceNumber = 0; // sequence 0 indicated error, 1 is starting
    unsigned uWAVHeaderSize = 44;
    unsigned uDatagramHeaderSize = 16; // NEEDS TO RESULT IN DATA WITH MULTIPLE OF 4 BYTES
    uint8_t uTransmissionState = 0;    // 0 - error 1 - active transmission 2 - complete
    unsigned uDataBytesTransmitted = 0;
    unsigned uTransmissionSize = 0;
    unsigned uMaxTranssionSize = 512; // bytes
    bool bTransmit = true;

    while (bTransmit)
    {

        if (uSequenceNumber == 0)
        {
            uTransmissionSize = uDatagramHeaderSize + uWAVHeaderSize;

            uint8_t auUDPData[uTransmissionSize] = {(uint8_t)0};
            uint8_t auWAVHeader[44] = {(uint8_t)0};
            ConvertHeaderToByteArray(psWAVFile->sWavHeader, auWAVHeader);

            // Filling for data to be transmitted
            memcpy(&auUDPData[0], &uSequenceNumber, sizeof(uSequenceNumber)); // 4 bytes
            memcpy(&auUDPData[4], &uTransmissionState, sizeof(uTransmissionState));
            memcpy(&auUDPData[5], &uTransmissionSize, sizeof(uTransmissionSize));
            memcpy(&auUDPData[uDatagramHeaderSize], &auWAVHeader, sizeof(auWAVHeader));

            const void *ptr_payload(&(auUDPData));
            int err = sendto(m_sock, ptr_payload, sizeof(auUDPData), 0, (struct sockaddr *)&m_dest_addr, sizeof(m_dest_addr));
            
            if (err < 0)
                ESP_LOGE(m_TAG, "Error occurred during sending: errno %d ", errno);

            // sBytesTransmitted = sBytesTransmitted + uTransmissionSize - uDatagramHeaderSize;
            uSequenceNumber++;
        }
        else
        {
            // Checking if need to adjust last transmission size
            if (uDataBytesTransmitted + uTransmissionSize - uDatagramHeaderSize > dTransmittableBytes)
            {
                uTransmissionSize = dTransmittableBytes - uDataBytesTransmitted + uDatagramHeaderSize;
                uTransmissionState = 1;
                bTransmit = false;
            }
            else
                uTransmissionSize = uMaxTranssionSize;

            uint8_t auUDPData[uTransmissionSize] = {(uint8_t)0};

            memcpy(&auUDPData[0], &uSequenceNumber, sizeof(uSequenceNumber)); // 4 bytes
            memcpy(&auUDPData[4], &uTransmissionState, sizeof(uTransmissionState));
            memcpy(&auUDPData[5], &uTransmissionSize, sizeof(uTransmissionSize));
            memcpy(&auUDPData[uDatagramHeaderSize], &(psWAVFile->vfWavData[uDataBytesTransmitted / 4]), uTransmissionSize - uDatagramHeaderSize);

            const void *ptr_payload(&(auUDPData));
            int err = sendto(m_sock, ptr_payload, uTransmissionSize, 0, (struct sockaddr *)&m_dest_addr, sizeof(m_dest_addr));

            if (err < 0)
                ESP_LOGE(m_TAG, "Error occurred during sending: errno %d ", errno);

            uDataBytesTransmitted = uDataBytesTransmitted + uTransmissionSize - uDatagramHeaderSize;
            uSequenceNumber++;
        }
    }
}

WAVFile WifiModule::ConvertTimeChunkToWAV(std::shared_ptr<TimeChunk> pTimeChunk)
{

    WAVFile sWavFile;

    // Creating WAV header
    // RIFF, WAVE, FMT, Subchunk2ID
    // Setting header parameters
    sWavFile.sWavHeader.SamplesPerSec = pTimeChunk->m_dSampleRate;
    sWavFile.sWavHeader.NumOfChan = pTimeChunk->m_uNumChannels;
    sWavFile.sWavHeader.bitsPerSample = pTimeChunk->m_uNumBytes * 8;
    sWavFile.sWavHeader.SamplesPerSec = pTimeChunk->m_dSampleRate;
    sWavFile.sWavHeader.blockAlign = 4; // stereo?
    // Setting chunk Sizes
    sWavFile.sWavHeader.Subchunk2Size = pTimeChunk->m_uNumChannels * pTimeChunk->m_dChunkSize * pTimeChunk->m_uNumBytes;
    sWavFile.sWavHeader.ChunkSize = sWavFile.sWavHeader.Subchunk2Size + 44 - 8;

    for (unsigned uSampleIndex = 0; uSampleIndex < pTimeChunk->m_dChunkSize; uSampleIndex++)
    {
        // Iterating through each ADC
        for (auto vADCData = pTimeChunk->m_vvvfTimeChunk.begin(); vADCData != pTimeChunk->m_vvvfTimeChunk.end(); ++vADCData)
        {
            // Iterating through each indivuidal ADC Channel
            for (auto vADCChannelData = vADCData->begin(); vADCChannelData != vADCData->end(); ++vADCChannelData)
            {
                // Pushing audio data onto wav data vector
                sWavFile.vfWavData.push_back((*vADCChannelData)[uSampleIndex]);
            }
        }
    }

    return sWavFile;
}

void WifiModule::ConvertHeaderToByteArray(WAVHeader sWAVHeader, uint8_t *arr)
{
    std::memcpy(&arr[0], &sWAVHeader.RIFF, sizeof(sWAVHeader.RIFF));                    // 0-3
    std::memcpy(&arr[4], &sWAVHeader.ChunkSize, sizeof(sWAVHeader.ChunkSize));          // 4-7
    std::memcpy(&arr[8], &sWAVHeader.WAVE, sizeof(sWAVHeader.WAVE));                    // 8-11
    std::memcpy(&arr[12], &sWAVHeader.fmt, sizeof(sWAVHeader.fmt));                     // 12-15
    std::memcpy(&arr[16], &sWAVHeader.Subchunk1Size, sizeof(sWAVHeader.Subchunk1Size)); // 16-19
    std::memcpy(&arr[20], &sWAVHeader.AudioFormat, sizeof(sWAVHeader.AudioFormat));     // 20-21
    std::memcpy(&arr[22], &sWAVHeader.NumOfChan, sizeof(sWAVHeader.NumOfChan));         // 22-23
    std::memcpy(&arr[24], &sWAVHeader.SamplesPerSec, sizeof(sWAVHeader.SamplesPerSec)); // 24-27
    std::memcpy(&arr[28], &sWAVHeader.bytesPerSec, sizeof(sWAVHeader.bytesPerSec));     // 28-31
    std::memcpy(&arr[32], &sWAVHeader.blockAlign, sizeof(sWAVHeader.blockAlign));       // 32-33
    std::memcpy(&arr[34], &sWAVHeader.bitsPerSample, sizeof(sWAVHeader.bitsPerSample)); // 34-35
    std::memcpy(&arr[36], &sWAVHeader.Subchunk2ID, sizeof(sWAVHeader.Subchunk2ID));     // 36-40
    std::memcpy(&arr[41], &sWAVHeader.Subchunk2Size, sizeof(sWAVHeader.Subchunk2Size)); // 41-44
}