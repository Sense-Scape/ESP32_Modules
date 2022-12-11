
#ifndef I2SMODULE
#define I2SIMODULE

/*Standard Includes*/
#include <memory>
#include <vector>
#include <complex>

/*Custom Includes*/
#include "BaseModule.h"
#include "TimeChunk.h"

/*ESP32 Includes*/
#include "driver/i2s.h"

/**
 * @brief Module responsible for interfacing with I2S audio sensors
 *
 */
class I2SModule : public BaseModule
{

public:

    /**
     *
     * @brief Construct a new Signal Processing Module object
     *
     * @param[in] uSampleRate Simulated sample rate in Hz
     * @param[in] uChunkSize Number of sampels in a single channel of chunk data
     * @param[in] uBufferSize Input buffer size
     * @param[in] uNumChannels Number of audio channels to sample
     */
    I2SModule(unsigned uSampleRate, unsigned uChunkSize, unsigned uNumChannels, unsigned uBufferSize);

    void Process(std::shared_ptr<BaseChunk> pBaseChunk) override;

    /**
     * 
     * @brief Check input buffer and try process data
     *
     */
    void ContinuouslyTryProcess() override;


private:
    i2s_config_t m_i2s_config_t ;               ///< ESP32 I2S config structure
    i2s_pin_config_t m_pin_config;
    unsigned m_uSampleRate;                     ///< I2S Sampling rate
    unsigned m_uChunkSize;                      ///< Single channel chunk size
    unsigned m_uNumChannels;                    ///< Number of audio channels to sample
    std::shared_ptr<TimeChunk> m_pTimeChunk;    ///< Pointer to member time data chunk
    QueueHandle_t m_pI2S_Queue;
    
    /**
     * 
     * @brief Creates and configures I2S sampling
    */
    void ConfigureI2S();

    /**
     * @brief Initializes Time Chunk vectrs default values. Initializes according to number of ADCs and their channels
     *
     */
    void ReinitializeTimeChunk();

};

#endif
