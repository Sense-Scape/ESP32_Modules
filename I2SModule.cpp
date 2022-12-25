#include "I2SModule.h"

I2SModule::I2SModule(unsigned uSampleRate, unsigned uChunkSize, unsigned uNumChannels, unsigned uBufferSize) :
BaseModule(uBufferSize),
m_uSampleRate(uSampleRate),
m_uChunkSize(uChunkSize),
m_uNumChannels(uNumChannels)
{
    ReinitializeTimeChunk();
    ConfigureI2S();
}

void I2SModule::ConfigureI2S()
{
    

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = m_uSampleRate,
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

    if (ESP_OK != i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL)) {
        std::cout << std::string(__PRETTY_FUNCTION__) + "i2s_driver_install: error";
    }

     if (m_pI2S_Queue == nullptr)
    {
        std::cout << std::string(__PRETTY_FUNCTION__) + "Failed to setup i2s event queue." << std::endl;;
    }


    i2s_pin_config_t pin_config = {
        .bck_io_num = 32,   // Bit Clock.
        .ws_io_num = 25,    // Word Select aka left/right clock aka LRCL.
        .data_out_num = -1,
        .data_in_num = 33,  // Data-out of the mic. (someone used 23 on forums).
    };

    m_pin_config = pin_config;

    if (ESP_OK != i2s_set_pin(I2S_NUM_0, &pin_config)) {
         std::cout << std::string(__PRETTY_FUNCTION__) + "i2s_set_pin: error";
    }

    ESP_ERROR_CHECK( i2s_set_clk(I2S_NUM_0, m_uSampleRate, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO) );

    i2s_start(I2S_NUM_0);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void I2SModule::Process(std::shared_ptr<BaseChunk> pBaseChunk)
{

    size_t bytesRead = 0;
    static uint16_t buffer16[256] = {0};

    while (true)
    {
		// Creating simulated data
        ReinitializeTimeChunk();

        i2s_read(I2S_NUM_0, &buffer16, 2*256, &bytesRead, 100);

        int samplesRead = bytesRead / 2;
        for (int i = 0; i < samplesRead; i++) 
        {
            //buffer32[i];  //(buffer32[i] << 16) |  (buffer32[i+1] << 8) | buffer32[i+2];
            //std::cout << std::to_string(buffer32[i]) << std::endl;
            m_pTimeChunk->m_vvfTimeChunks[0][i] = buffer16[i];//<XX*( 3.3 / (std::pow(2, 16) - 1)) - 3.3/2);
        }

        // Passing data on
        std::shared_ptr<TimeChunk> pTimeChunk = m_pTimeChunk;
        if (!TryPassChunk(pTimeChunk))
        {
            std::cout << std::string(__PRETTY_FUNCTION__) + ": Next buffer full, dropping current chunk and passing \n";
        }
        
        //TODO: Add a means to exit this in the case that this thread needs to be killed
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
    m_pTimeChunk = std::make_shared<TimeChunk>(m_uChunkSize, m_uSampleRate, 0, 12, sizeof(float), 1);
    m_pTimeChunk->m_vvfTimeChunks.resize(m_uNumChannels);

    unsigned uADCChannelCount = 0;

    // Current implementation simulated N channels on a single ADC
    for (unsigned uADCChannel = 0; uADCChannel < m_uNumChannels; uADCChannel++)
    {
        // Initialising channel data vector for each ADC
        m_pTimeChunk->m_vvfTimeChunks[uADCChannel].resize(m_uChunkSize);
        uADCChannelCount++;
    }

    m_pTimeChunk->m_uNumChannels = m_uNumChannels;
}
