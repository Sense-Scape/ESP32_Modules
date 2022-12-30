
#ifndef I2SMODULE
#define I2SIMODULE

/* Standard Includes */
#include <memory>
#include <vector>
#include <complex>

/* Custom Includes */
#include "BaseModule.h"
#include "TimeChunk.h"

/* ESP32 Includes */
#include "driver/i2s.h"

/**@brief Structure to encapsulate configuration of I2S pins*/
struct I2SPinConfig
{
    int uWordSelect = -1;   ///< Word select pin
    int uClock = -1;        ///< Clock Pin
    int uDataIn = -1;       ///< Data in Pin    
    int uDataOut = -1;      ///< Data out pin
    unsigned uInterface;    ///< I2S Interface to use    
    bool bLeft = true;      ///< Left or right channel
};

/**@brief Structure to encapsulate configuration of the I2S module*/
struct I2SModuleConfig
{
    unsigned m_uSampleRate;                     ///< I2S Sampling rate
    unsigned m_uChunkSize;                      ///< Single channel chunk size
    unsigned m_uNumChannels;                    ///< Number of audio channels to sample (size of m_vI2SPinConfig)
    std::vector<I2SPinConfig> m_vI2SPinConfig;  ///< Vector of pin configurations
};

/**
 * @brief Module responsible for interfacing with I2S audio sensors
 */
class I2SModule : public BaseModule
{
public:

    /**
     * @brief Construct a new Signal Processing Module object
     * @param[in] I2SModuleConfig I2SModuleConfig structure used to configure the IS2 module
     */
    I2SModule(I2SModuleConfig sI2SModuleConfig, unsigned uBufferSize);

    /**
     * @brief The loop process the I2S module completes
     * @param[in] pBaseChunk pointer to base chunk
     */
    void Process(std::shared_ptr<BaseChunk> pBaseChunk) override;

    /**
     * @brief Check input buffer and try process data
     */
    void ContinuouslyTryProcess() override;

private:
    I2SModuleConfig m_sI2SModuleConfig;                         ///< I2SModuleConfig structure used to configure the IS2 module
    i2s_config_t m_i2s_config_t;                ///< ESP32 I2S config structure
    std::vector<i2s_pin_config_t> m_v_pin_config;
    std::shared_ptr<TimeChunk> m_pTimeChunk;    ///< Pointer to member time data chunk
    
    /**
     * @brief Creates and configures I2S sampling
    */
    void ConfigureI2S();

    /**
     * @brief Initializes Time Chunk vectrs default values. Initializes according to number of ADCs and their channels
     */
    void ReinitializeTimeChunk();

};

#endif
