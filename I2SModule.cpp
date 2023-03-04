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
    // i2s_config_t i2s_config = {
    //     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    //     .sample_rate = m_sI2SModuleConfig.m_uSampleRate,
    //     .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    //     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Ground the L/R pin on the INMP441.
    //     .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
    //     .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    //     .dma_buf_count = 8,
    //     .dma_buf_len = (128*4)*2,
    //     .use_apll = true,
    //     .tx_desc_auto_clear = false,
    //     .fixed_mclk = 0,
    //     };
    // m_i2s_config_t = i2s_config;

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX |I2S_MODE_PDM),
        .sample_rate = m_sI2SModuleConfig.m_uSampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Ground the L/R pin on the INMP441.
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_PCM),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = (128*4)*2,
        .use_apll = true,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
        };
    m_i2s_config_t = i2s_config;


// m_sI2SModuleConfig.m_vI2SPinConfig.size()
    for (unsigned uI2SInterfaceIndex = 0; uI2SInterfaceIndex < 1; uI2SInterfaceIndex++)
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

        // if (ESP_OK != i2s_set_pin((i2s_port_t)uI2SInterfaceIndex, &m_v_pin_config[uI2SInterfaceIndex])) {
        //     std::cout << std::string(__PRETTY_FUNCTION__) + "i2s_set_pin: error";
        // }


        i2s_driver_install((i2s_port_t)uI2SInterfaceIndex, &i2s_config, 0, NULL);
        i2s_set_pin((i2s_port_t)uI2SInterfaceIndex, &pin_config);
    
        
    }

    //ESP_ERROR_CHECK( i2s_set_clk((i2s_port_t)0, m_sI2SModuleConfig.m_uSampleRate, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO) );

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void I2SModule::Process(std::shared_ptr<BaseChunk> pBaseChunk)
{

    size_t bytesRead = 0;
    static uint8_t buffer8[128*4*2];
    int samplesRead = 0;

    std::unique_lock<std::mutex> ProcessLock(m_ProcessStateMutex);
    
    while (!m_bShutDown)
    {
        ProcessLock.unlock();
        ReinitializeTimeChunk();
        

        i2s_read((i2s_port_t)0, &buffer8, (128*4)*2, &bytesRead, 1000);

        // Storing samples
        samplesRead = bytesRead / 4;
        
        //std::cout<< " --- " std::to_string(buffer8[102]) << std::endl;
        // std::cout<< " --- " + std::to_string(buffer8[1022]) << std::endl;
        // std::cout<< " --- " + std::to_string(buffer8[1023]) << std::endl;

        for (int i = 0; i < samplesRead/2; i++) 
        {   
            // 4*(2*i) is the start of channel 1 data. There are 4 bytes of "data" after
            // 4*(2*i + 1 ) is the start of channel 2 data.
            m_pTimeChunk->m_vvi16TimeChunks[0][i] = (buffer8[4*(2*i) + 2 ] << 8) | (buffer8[4*(2*i) + 3 ]); 
            m_pTimeChunk->m_vvi16TimeChunks[1][i] = (buffer8[4*(2*i + 1) + 2] << 8) | (buffer8[4*(2*i + 1) + 3]); 

            // std::cout<< std::to_string(i) + " --- " + std::to_string(m_pTimeChunk->m_vvi16TimeChunks[0][i]) << std::endl;
            // std::cout<< std::to_string(i) + " --- " + std::to_string(m_pTimeChunk->m_vvi16TimeChunks[1][i]) << std::endl;
        }

        // std::cout<< " --- " + std::to_string(m_pTimeChunk->m_vvi16TimeChunks[1][127]) << std::endl;
        // std::cout<< " --- " + std::to_string(m_pTimeChunk->m_vvi16TimeChunks[0][127]) << std::endl;

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
