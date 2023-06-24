#include "I2SModule.h"

I2SModule::I2SModule(I2SModuleConfig sI2SModuleConfig, unsigned uBufferSize) :
BaseModule(uBufferSize),
m_sI2SModuleConfig(sI2SModuleConfig),
m_i2s_chan_handle_t(ConfigureI2S())
{
    ReinitializeTimeChunk();
}

i2s_chan_handle_t I2SModule::ConfigureI2S()
{
    i2s_chan_handle_t rx_chan;
    /* Setp 1: Determine the I2S channel configuration and allocate RX channel only
     * The default configuration can be generated by the helper macro,
     * but note that PDM channel can only be registered on I2S_NUM_0 */
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    /* Step 2: Setting the configurations of PDM RX mode and initialize the RX channel
     * The slot configuration and clock configuration can be generated by the macros
     * These two helper macros is defined in 'i2s_pdm.h' which can only be used in PDM RX mode.
     * They can help to specify the slot and clock configurations for initialization or re-configuring */
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = {
            .sample_rate_hz = m_sI2SModuleConfig.m_uSampleRate,
            .clk_src = I2S_CLK_SRC_APLL,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            .dn_sample_mode = I2S_PDM_DSR_8S 
        },//I2S_PDM_RX_CLK_DEFAULT_CONFIG(m_sI2SModuleConfig.m_uSampleRate),
        /* The data bit-width of PDM mode is fixed to 16 */
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .clk = (gpio_num_t)m_sI2SModuleConfig.m_vI2SPinConfig[0].uClock,
            .din = (gpio_num_t)m_sI2SModuleConfig.m_vI2SPinConfig[0].uDataIn,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_chan, &pdm_rx_cfg));

    /* Step 3: Enable the rx channels before reading data */
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return rx_chan;
}

void I2SModule::Process(std::shared_ptr<BaseChunk> pBaseChunk)
{

    size_t bytesRead = 0;
    static uint8_t buffer8[128*2*2];
    
    int samplesRead = 0;

    while (!m_bShutDown)
    {
        
        // Comment: Do not actually have to do this ass data will be overwritten anyway
        ReinitializeTimeChunk();

        ESP_ERROR_CHECK(i2s_channel_read(m_i2s_chan_handle_t, &buffer8, (128*2)*2, &bytesRead, 1000));

        // Update time as time since boot in microseconds
        m_pTimeChunk->m_i64TimeStamp = esp_timer_get_time();

        samplesRead = bytesRead/2;

        uint8_t* pbuffer8Chan1 = &buffer8[0];
        uint8_t* pbuffer8Chan2 = &buffer8[2];

        for (int i = 0; i < samplesRead/2; i++) 
        {   
            m_pTimeChunk->m_vvi16TimeChunks[0][i] = (*pbuffer8Chan1) | ((*(pbuffer8Chan1+1)) << 8);
            m_pTimeChunk->m_vvi16TimeChunks[1][i] = (*pbuffer8Chan2) | ((*(pbuffer8Chan2+1)) << 8);

            pbuffer8Chan1 = pbuffer8Chan1+2;
            pbuffer8Chan2 = pbuffer8Chan2+2;
        }

        // Passing data on
        std::shared_ptr<TimeChunk> pTimeChunk = std::move(m_pTimeChunk);

        if (!TryPassChunk(pTimeChunk))
            std::cout << std::string(__PRETTY_FUNCTION__) + ": Next buffer full, dropping current chunk and passing \n";
    }

}

void I2SModule::ContinuouslyTryProcess()
{
    while (!m_bShutDown)
    {
        auto pBaseChunk = std::make_shared<BaseChunk>();
        Process(pBaseChunk);
    }
}

void I2SModule::ReinitializeTimeChunk()
{
    // Assigning variables for shorter line
    auto uChunkSize = m_sI2SModuleConfig.m_uChunkSize;
    auto uChannelSampleRate = m_sI2SModuleConfig.m_uSampleRate;
    auto uNumChannels = m_sI2SModuleConfig.m_uNumChannels;

    m_pTimeChunk = std::make_shared<TimeChunk>(uChunkSize, uChannelSampleRate, 0, sizeof(uint16_t)*8, sizeof(uint16_t), 1);
    m_pTimeChunk->m_vvi16TimeChunks.resize(uNumChannels);

    // Current implementation simulated N channels on a single ADC
    for (unsigned uChannel = 0; uChannel < uNumChannels; uChannel++)
    {
        // Initialising channel data vector for each ADC
        m_pTimeChunk->m_vvi16TimeChunks[uChannel].resize(uChunkSize);
    }

    m_pTimeChunk->m_uNumChannels = uNumChannels;
}