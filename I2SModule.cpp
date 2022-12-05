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
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // Ground the L/R pin on the INMP441.
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = 256 * 4,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        };

    m_i2s_config_t = i2s_config;

    
    if (ESP_OK != i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL)) {
        std::cout << std::string(__PRETTY_FUNCTION__) + "i2s_driver_install: error";
    }

    i2s_pin_config_t pin_config = {
        .bck_io_num = 26,   // Bit Clock.
        .ws_io_num = 25,    // Word Select aka left/right clock aka LRCL.
        .data_out_num = -1,
        .data_in_num = 23,  // Data-out of the mic. (someone used 23 on forums).
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
    while (true)
    {
		// Creating simulated data
        ReinitializeTimeChunk();
        
        size_t bytesRead = 0;
        uint8_t buffer32[256 * 4] = {0};
        i2s_read(I2S_NUM_0, &buffer32, sizeof(buffer32), &bytesRead, 100);

        int samplesRead = bytesRead / 4;


        for (int i = 0; i < samplesRead; i++) 
        {
            uint8_t mid = buffer32[i * 4 + 2];
            uint8_t msb = buffer32[i * 4 + 3];
            uint16_t raw = (((uint32_t)msb) << 8) + ((uint32_t)mid);
            m_pTimeChunk->m_vvvdTimeChunk[0][0][i] = raw;
            // memcpy(&buffer16[i], &raw, sizeof(raw)); // Copy so sign bits aren't interfered with somehow.
        }

		// Passing data on
		std::shared_ptr<TimeChunk> pTimeChunk = std::move(m_pTimeChunk);
		if (!TryPassChunk(std::static_pointer_cast<BaseChunk>(pTimeChunk)))
		{
			std::cout << std::string(__PRETTY_FUNCTION__) + ": Next buffer full, dropping current chunk and passing \n";
		}

        // Sleeping for time equivalent to chunk period
        std::cout << std::string(__PRETTY_FUNCTION__) + ": I2S chunk read and passed on \n";

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
    m_pTimeChunk = std::make_shared<TimeChunk>(m_uChunkSize, m_uSampleRate, 0, 12, sizeof(float));
    m_pTimeChunk->m_vvvdTimeChunk.resize(1);
    m_pTimeChunk->m_vvvdTimeChunk[0].resize(m_uNumChannels);

    unsigned uADCChannelCount = 0;

    // Current implementation simulated N channels on a single ADC
    for (unsigned uADCChannel = 0; uADCChannel < m_uNumChannels; uADCChannel++)
    {
        // Initialising channel data vector for each ADC
        m_pTimeChunk->m_vvvdTimeChunk[0][uADCChannel].resize(m_uChunkSize);
        uADCChannelCount++;
    }

    m_pTimeChunk->m_uNumChannels = m_uNumChannels;
}
