#ifndef WIFIMODULE
#define WIFIMODULE

#include <map>
#include <cmath>
#include <thread>
#include <memory>

#include "BaseModule.h"
#include "TimeChunk.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_task_wdt.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#define ESP_MAXIMUM_RETRY 3

typedef struct WAVFile
{
    WAVHeader sWavHeader;           ///< WAV header
    std::vector<float> vfWavData;   ///< Multi channel packed WAV data
} WAVEFile;


/**
 * @brief Module responsible for conversion to WAV structure and transmission of data to server
 */
class WifiModule : public BaseModule
{
public:
    static EventGroupHandle_t m_s_wifi_event_group; ///< Required by ESP-IDF
    static const char *m_TAG;                       ///< Required by ESP-IDF
    static int s_retry_num;                         ///< Required by ESP-IDF - connection retires before failure

    /**
     * @brief Construct a new Signal Processing Module object
     * @param[in] m_sSSID SSID of WiFi network
     * @param[in] m_sPassword Password of WiFi network
     * @param[in] sHostIPAddress IP address of target device
     * @param[in] uUDPport Port over which to stream
     * @param[in] uDatagramSize TX datagram size in bytes
     * @param[in] uBufferSize Size of input buffer
     */
    WifiModule(std::string m_sSSID, std::string m_sPassword, std::string sHostIPAddress, unsigned uUDPport, unsigned uDatagramSize, unsigned uBufferSize);
    //~WifiModule() = default;

    /**
     * @brief Wifi event handler function to control connection states
     * @param arg
     * @param event_base
     * @param event_id
     * @param event_data
     */
    static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

private:
    std::string m_sSSID;            ///< SSID of WiFi network
    std::string m_sPassword;        ///< Password of WiFi network
    std::string m_sHostIPAddress;   ///< IP address of host
    unsigned m_uUDPPort;            ///< UDP port that the ESP transmits to
    int m_sock;                     ///< ESP Wifi Socket
    sockaddr_in m_dest_addr;        ///> Destination IP address
    unsigned m_uDatagramSize;       ///> Datagram transmission size in bytes
    unsigned m_uSessionNumber; 

    /**
     * @brief Transmit message using UDP to webserver
     * @param pBaseChunk pointer to base chunk
     */
    void SendUDP(std::shared_ptr<BaseChunk> pBaseChunk);

    /**
     * @brief Connect to WiFi using module member parameters
     */
    void ConnectWifiConnection();

    /**
     * @brief Connect Wifi module to socket
     */
    void ConnectToSocket();

    /**
     * @brief The loop process the Wifi module completes
     * @param[in] pBaseChunk pointer to base chunk
     */
    void Process(std::shared_ptr<BaseChunk> pBaseChunk) override;
};

#endif