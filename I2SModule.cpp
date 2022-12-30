#include "I2SModule.h"

I2SModule::I2SModule(I2SModuleConfig sI2SModuleConfig, unsigned uBufferSize) :
BaseModule(uBufferSize),
m_sI2SModuleConfig(sI2SModuleConfig)
{
    ReinitializeTimeChunk();
    ConfigureI2S();
}

void I2SModule::ConfigureI2S()
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = m_sI2SModuleConfig.m_uSampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Ground the L/R pin on the INMP441.
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_MSB),
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = 256 * 2,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = false,
        };
    m_i2s_config_t = i2s_config;

    auto a = I2S_CHANNEL_MONO;
   if (m_sI2SModuleConfig.m_vI2SPinConfig.size() > 1)
        a = I2S_CHANNEL_STEREO;

for (unsigned uI2SInterfaceIndex = 0; uI2SInterfaceIndex < m_sI2SModuleConfig.m_vI2SPinConfig.size(); uI2SInterfaceIndex++)
    {   
        if (ESP_OK != i2s_driver_install((i2s_port_t)uI2SInterfaceIndex, &i2s_config, 0, NULL)) 
            std::cout << std::string(__PRETTY_FUNCTION__) + "i2s_driver_install: error";

        i2s_pin_config_t pin_config = {
            .bck_io_num = m_sI2SModuleConfig.m_vI2SPinConfig[uI2SInterfaceIndex].uClock,   // Bit Clock.
            .ws_io_num = m_sI2SModuleConfig.m_vI2SPinConfig[uI2SInterfaceIndex].uWordSelect,    // Word Select aka left/right clock aka LRCL.
            .data_out_num = m_sI2SModuleConfig.m_vI2SPinConfig[uI2SInterfaceIndex].uDataOut,
            .data_in_num = m_sI2SModuleConfig.m_vI2SPinConfig[uI2SInterfaceIndex].uDataIn,  // Data-out of the mic. (someone used 23 on forums).
        };

        m_v_pin_config.push_back(pin_config);

        if (ESP_OK != i2s_set_pin((i2s_port_t)uI2SInterfaceIndex, &m_v_pin_config[uI2SInterfaceIndex])) {
            std::cout << std::string(__PRETTY_FUNCTION__) + "i2s_set_pin: error";
        }
    
        ESP_ERROR_CHECK( i2s_set_clk((i2s_port_t)uI2SInterfaceIndex, m_sI2SModuleConfig.m_uSampleRate, I2S_BITS_PER_SAMPLE_32BIT, a) );
        i2s_start((i2s_port_t)uI2SInterfaceIndex);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void I2SModule::Process(std::shared_ptr<BaseChunk> pBaseChunk)
{

    size_t bytesRead = 0;
    static uint16_t buffer16[256] = {0};
    std::unique_lock<std::mutex> ProcessLock(m_ProcessStateMutex);
    
    while (!m_bShutDown)
    {
        ProcessLock.unlock();
		// Getting I2S Data
        ReinitializeTimeChunk();
    
        for (unsigned uI2SInterfaceIndex = 0; uI2SInterfaceIndex < m_sI2SModuleConfig.m_vI2SPinConfig.size(); uI2SInterfaceIndex++)
        {   
            i2s_read((i2s_port_t)uI2SInterfaceIndex, &buffer16, 2*256, &bytesRead, 100);

            // Storing samples
            int samplesRead = bytesRead / 2;
            for (int i = 0; i < samplesRead; i++) 
               m_pTimeChunk->m_vvfTimeChunks[uI2SInterfaceIndex][i] = buffer16[i]; // WARNING: I2S interface will change if not 1 mic per channel
        }        
        
        // Passing data on
        std::shared_ptr<TimeChunk> pTimeChunk = std::move(m_pTimeChunk);

        if (!TryPassChunk(pTimeChunk))
            std::cout << std::string(__PRETTY_FUNCTION__) + ": Next buffer full, dropping current chunk and passing \n";
        
        ProcessLock.lock();
    }

}

void I2SModule::ContinuouslyTryProcess()
{
    std::unique_lock<std::mutex> ProcessLock(m_ProcessStateMutex);

    while (!m_bShutDown)
    {
        ProcessLock.unlock();
        auto pBaseChunk = std::make_shared<BaseChunk>();
        Process(pBaseChunk);

        ProcessLock.lock();
    }
}

void I2SModule::ReinitializeTimeChunk()
{
    // Assigning variables for shorter line
    auto uChunkSize = m_sI2SModuleConfig.m_uChunkSize;
    auto uSampleRate = m_sI2SModuleConfig.m_uSampleRate;
    auto uNumChannels = m_sI2SModuleConfig.m_uNumChannels;

    m_pTimeChunk = std::make_shared<TimeChunk>(uChunkSize, uSampleRate, 0, sizeof(uint16_t)*8, sizeof(uint16_t), 1);
    m_pTimeChunk->m_vvfTimeChunks.resize(uNumChannels);

    // Current implementation simulated N channels on a single ADC
    for (unsigned uChannel = 0; uChannel < uNumChannels; uChannel++)
    {
        // Initialising channel data vector for each ADC
        m_pTimeChunk->m_vvfTimeChunks[uChannel].resize(uChunkSize);
    }

    m_pTimeChunk->m_uNumChannels = uNumChannels;
}
