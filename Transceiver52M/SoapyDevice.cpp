//
// Copyright (c) 2015-2015 AB Open Ltd
// SPDX-License-Identifier: BSL-1.0
//

#include "radioDevice.h"
#include "Threads.h"
#include "Logger.h"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <sched.h> //thread prio

/***********************************************************************
 * Device specific settings
 **********************************************************************/
struct DeviceSpecificSettings
{
    std::string key;
    size_t sps;
    double rxOffset; //rx adjustment in seconds
    double clockRate; //non 0 to set master clock rate
    bool sharedLo;
    std::string rxFeMap; //sets RX frontend map config when specified
    std::string txFeMap; //sets TX frontend map config when specified
    std::string desc;
};

static DeviceSpecificSettings settings[] = {
    { "umtrx-rev0", 1, 9.9692e-5, 0.0, false, "A:0 B:0", "A:0 B:0", "UmTRX 1 SPS" },
    { "umtrx-rev0", 4, 7.3846e-5, 0.0, false, "A:0 B:0", "A:0 B:0", "UmTRX 4 SPS" },
};

/***********************************************************************
 * SoapyDevice overload
 **********************************************************************/
class SoapyDevice : public RadioDevice
{
public:
  SoapyDevice(size_t sps, size_t chans, bool diversity, double offset);

  ~SoapyDevice(void);

  /** Initialize the Device */
  int open(const std::string &args = "", bool extref = false);

  /** Start the Device */
  bool start();

  /** Stop the Device */
  bool stop();

  /** Get the Tx window type */
  enum TxWindowType getWindowType()
  {
      return TX_WINDOW_FIXED;
  }

  /** Enable thread priority */
  void setPriority(float prio);

  /**
	Read samples from the radio.
	@param buf preallocated buf to contain read result
	@param len number of samples desired
	@param overrun Set if read buffer has been overrun, e.g. data not being read fast enough
	@param timestamp The timestamp of the first samples to be read
	@param underrun Set if radio does not have data to transmit, e.g. data not being sent fast enough
	@param RSSI The received signal strength of the read result
	@return The number of samples actually read
  */
  int readSamples(std::vector<short *> &bufs, int len, bool *overrun,
                          TIMESTAMP timestamp, bool *underrun,
                          unsigned *RSSI);
  /**
        Write samples to the radio.
        @param buf Contains the data to be written.
        @param len number of samples to write.
        @param underrun Set if radio does not have data to transmit, e.g. data not being sent fast enough
        @param timestamp The timestamp of the first sample of the data buffer.
        @param isControl Set if data is a control packet, e.g. a ping command
        @return The number of samples actually written
  */
  int writeSamples(std::vector<short *> &bufs, int len, bool *underrun,
                           TIMESTAMP timestamp, bool isControl);

  void flushRxSamples(void);

  /** Update the alignment between the read and write timestamps */
  bool updateAlignment(TIMESTAMP timestamp)
  {
      return true; //assume always aligned
  }

  /** Set the transmitter frequency */
  bool setTxFreq(double wFreq, size_t chan);

  /** Set the receiver frequency */
  bool setRxFreq(double wFreq, size_t chan);

  bool setFrequency(const int dir, double wFreq, size_t chan);

  /** Returns the starting write Timestamp*/
  TIMESTAMP initialWriteTimestamp(void)
  {
      return _initialTime * _sps;
  }

  /** Returns the starting read Timestamp*/
  TIMESTAMP initialReadTimestamp(void)
  {
      return _initialTime;
  }

  /** returns the full-scale transmit amplitude **/
  double fullScaleInputValue()
  {
      return 32000*0.3;
  }

  /** returns the full-scale receive amplitude **/
  double fullScaleOutputValue()
  {
      return 32000;
  }

  /** sets the receive chan gain, returns the gain setting **/
  double setRxGain(double dB, size_t chan);

  /** gets the current receive gain **/
  double getRxGain(size_t chan);

  /** return maximum Rx Gain **/
  double maxRxGain(void);

  /** return minimum Rx Gain **/
  double minRxGain(void);

  /** sets the transmit chan gain, returns the gain setting **/
  double setTxGain(double dB, size_t chan);

  /** return maximum Tx Gain **/
  double maxTxGain(void);

  /** return minimum Tx Gain **/
  double minTxGain(void);

  /** Return internal status values */
  double getTxFreq(size_t chan);
  double getRxFreq(size_t chan);
  double getSampleRate();
  double numberRead();
  double numberWritten();

private:
    DeviceSpecificSettings _settings;
    SoapySDR::Device *_device;
    SoapySDR::Stream *_rxStream;
    SoapySDR::Stream *_txStream;
    std::vector<size_t> _chans;
    const size_t _sps;
    const bool _diversityEnabled;
    const double _tuningOffset;

    //stats
    long long _numReadden;
    long long _numWritten;

    //rx time tracking
    TIMESTAMP _initialTime;
    TIMESTAMP _rxTicksOffset;
    TIMESTAMP _nextRxTime;

    //actual rates after setting sample rates
    double _actualRxRate;
    double _actualTxRate;

    TIMESTAMP timeNsToRxTicks(const long long timeNs)
    {
        return TIMESTAMP(timeNs*(_actualRxRate/1e9));
    }

    long long txTicksToTimeNs(const TIMESTAMP ticks)
    {
        return ticks*(1e9/_actualTxRate);
    }
};

/***********************************************************************
 * Logger hooks
 **********************************************************************/
static void SoapyDevice_logger(const SoapySDR::LogLevel logLevel, const char *message)
{
    switch(logLevel)
    {
    case SOAPY_SDR_FATAL:    LOG(EMERG)   << message; break;
    case SOAPY_SDR_CRITICAL: LOG(CRIT)    << message; break;
    case SOAPY_SDR_ERROR:    LOG(ERR)     << message; break;
    case SOAPY_SDR_WARNING:  LOG(WARNING) << message; break;
    case SOAPY_SDR_NOTICE:   LOG(NOTICE)  << message; break;
    case SOAPY_SDR_INFO:     LOG(INFO)    << message; break;
    case SOAPY_SDR_DEBUG:    LOG(DEBUG)   << message; break;
    case SOAPY_SDR_TRACE:    LOG(DEBUG)   << message; break;
    }
}

/***********************************************************************
 * thread prio
 **********************************************************************/
static std::string _setPriority(const double prio)
{
    //no negative priorities supported on this OS
    if (prio <= 0.0) return "";

    //determine priority bounds
    const int policy(SCHED_RR);
    const int maxPrio = sched_get_priority_max(policy);
    if (maxPrio < 0) return strerror(errno);
    const int minPrio = sched_get_priority_min(policy);
    if (minPrio < 0) return strerror(errno);

    //set realtime priority and prio number
    struct sched_param param;
    std::memset(&param, 0, sizeof(param));
    param.sched_priority = minPrio + int(prio * (maxPrio-minPrio));
    if (sched_setscheduler(0, policy, &param) != 0) return strerror(errno);

    return "";
}

/***********************************************************************
 * SoapyDevice implementation
 **********************************************************************/
SoapyDevice::SoapyDevice(size_t sps, size_t chans, bool diversity, double offset):
    _device(NULL),
    _rxStream(NULL),
    _txStream(NULL),
    _sps(sps),
    _diversityEnabled(diversity),
    _tuningOffset(offset),
    _numReadden(0),
    _numWritten(0)
{
    for (size_t i = 0; i < chans; i++) _chans.push_back(i);
}

SoapyDevice::~SoapyDevice(void)
{
    LOG(INFO) << "Cleanup SoapySDR device";
    if (_rxStream != NULL) _device->closeStream(_rxStream);
    if (_txStream != NULL) _device->closeStream(_txStream);
    SoapySDR::Device::unmake(_device);
}

int SoapyDevice::open(const std::string &args, bool extref)
{
    LOG(INFO) << "Make SoapySDR device " << args;
    _device = SoapySDR::Device::make(args);

    //check for hardware time support
    if (not _device->hasHardwareTime())
    {
        LOG(ERR) << "No hardware time support for " << _device->getHardwareKey();
        return -1;
    }

    //set option external reference
    if (extref) _device->setClockSource("EXTERNAL");

    //create streams
    _rxStream = _device->setupStream(SOAPY_SDR_RX, "CS16", _chans);
    _txStream = _device->setupStream(SOAPY_SDR_TX, "CS16", _chans);
    LOG(INFO) << "Using " << _chans.size() << " channels";

    //discover the settings
    LOG(INFO) << "Looking for device specific settings for: " << _device->getHardwareKey();
    bool found = false;
    _settings.clockRate = 0.0;
    _settings.rxOffset = 0.0;
    _settings.sharedLo = false;
    for (size_t i = 0; i < sizeof(settings)/sizeof(DeviceSpecificSettings); i++)
    {
        if (strcasecmp(settings[i].key.c_str(), _device->getHardwareKey().c_str()) != 0) continue;
        if (settings[i].sps != _sps) continue;
        _settings = settings[i];
        found = true;
    }
    if (not found)
    {
        LOG(ERR) << "No device specific settings match for " << _device->getHardwareKey();
    }

    //set frontend mapping if specified
    if (not _settings.rxFeMap.empty()) _device->setFrontendMapping(SOAPY_SDR_RX, _settings.rxFeMap);
    if (not _settings.txFeMap.empty()) _device->setFrontendMapping(SOAPY_SDR_TX, _settings.txFeMap);

    //set the clock rate if specified
    if (_settings.clockRate != 0.0)
    {
        LOG(INFO) << "setting the clock rate " << (_settings.clockRate/1e6) << " MHz";
        _device->setMasterClockRate(_settings.clockRate);
    }

    //set the sample rate
    const double sampleRate = GSMRATE * _sps;
    for (size_t i = 0; i < _chans.size(); i++)
    {
        _device->setSampleRate(SOAPY_SDR_RX, _chans[i], sampleRate);
        _device->setSampleRate(SOAPY_SDR_TX, _chans[i], sampleRate);
    }

    //readback actual rates, used for time stamps
    _actualRxRate = _device->getSampleRate(SOAPY_SDR_RX, 0);
    _actualTxRate = _device->getSampleRate(SOAPY_SDR_TX, 0);
    LOG(INFO) << "RX sample rate " << (_actualRxRate/1e6) << " MHz";
    LOG(INFO) << "TX sample rate " << (_actualTxRate/1e6) << " MHz";

    _rxTicksOffset = _actualRxRate*_settings.rxOffset;
    LOG(INFO) << "RX ticks adjustment: " << _rxTicksOffset;
    return NORMAL;
}

bool SoapyDevice::start()
{
    LOG(INFO) << "Starting SoapySDR streams";
    _device->setHardwareTime(0);
    int ret = 0;
    const long deltaNs = long(1e8/*100ms*/);
    const long long timeStartNs = _device->getHardwareTime() + deltaNs;
    ret |= _device->activateStream(_rxStream, SOAPY_SDR_HAS_TIME, timeStartNs);
    ret |= _device->activateStream(_txStream);
    LOG(INFO) << "Started SoapySDR streams " << ret;

    //receive a few packets to establish time
    this->flushRxSamples();
    _initialTime = _nextRxTime;

    return ret == 0;
}

bool SoapyDevice::stop()
{
    LOG(INFO) << "Stopping SoapySDR streams";
    int ret = 0;
    ret |= _device->deactivateStream(_rxStream);
    ret |= _device->deactivateStream(_txStream);
    LOG(INFO) << "Stopped SoapySDR streams " << ret;
    return ret == 0;
}

void SoapyDevice::setPriority(float prio)
{
    std::string result = _setPriority(prio);
    if (result.empty()) return;
    LOG(WARNING) << "failed set prio " << result;
}

void SoapyDevice::flushRxSamples(void)
{
    std::complex<short> buff[4096];
    int flags = SOAPY_SDR_ONE_PACKET;
    long long timeNs;
    void *rxbuffs[_chans.size()];
    for (size_t i = 0; i < _chans.size(); i++) rxbuffs[i] = buff;
    int ret = _device->readStream(_rxStream, rxbuffs, 4096, flags, timeNs, long(1e6));

    //update concept of time whenever time is valid
    if (ret > 0 and (flags & SOAPY_SDR_HAS_TIME) != 0)
    {
        _nextRxTime = this->timeNsToRxTicks(timeNs) + ret;
    }
}

int SoapyDevice::readSamples(std::vector<short *> &bufs, int len, bool *overrun,
                          TIMESTAMP timestamp, bool *underrun,
                          unsigned *RSSI)
{
    timestamp += _rxTicksOffset;
    *overrun = false;
    *underrun = false;

    if (bufs.size() != _chans.size())
    {
        LOG(ERR) << "bufs.size() channel mismatch";
        return -1;
    }

    //read until the buffer is filled up
    size_t sampsTotal = 0;
    while (sampsTotal < size_t(len))
    {
        int flags = 0;
        long long timeNs;
        void *rxbuffs[bufs.size()];
        for (size_t i = 0; i < bufs.size(); i++) rxbuffs[i] = bufs[i] + sampsTotal*2;

        //determine receive size to strip off the early samples
        size_t numSampsRx = len-sampsTotal;
        size_t numSampsStrip = 0;
        if (sampsTotal == 0 and timestamp > _nextRxTime)
        {
            numSampsStrip = std::min<size_t>(len, (timestamp-_nextRxTime));
            numSampsRx = numSampsStrip;
        }

        //perform a receive
        int ret = _device->readStream(_rxStream, rxbuffs, numSampsRx, flags, timeNs, long(1e5));
        if (ret == SOAPY_SDR_TIMEOUT) return 0;
        if (ret == SOAPY_SDR_OVERFLOW) continue;
        if (ret < 0)
        {
            LOG(ERR) << "readStream ret=" << ret;
            return 0;
        }
        _numReadden += ret;

        //update concept of time whenever time is valid
        if ((flags & SOAPY_SDR_HAS_TIME) != 0)
        {
            _nextRxTime = this->timeNsToRxTicks(timeNs);
        }

        //check if the timestamp is late
        if (timestamp < _nextRxTime and sampsTotal == 0)
        {
            LOG(ERR) << "readStream late";
            return 0;
        }

        //update next rx time
        _nextRxTime += ret;

        //update the number of samples accumulated (throw out early)
        if (numSampsStrip != 0) sampsTotal = 0;
        else sampsTotal += ret;
    }

    return sampsTotal;
}

int SoapyDevice::writeSamples(std::vector<short *> &bufs, int len, bool *underrun,
                           TIMESTAMP timestamp, bool isControl)
{
    if (bufs.size() != _chans.size())
    {
        LOG(ERR) << "bufs.size() channel mismatch";
        return -1;
    }

    *underrun = false;

    //control packets not supported
    if (isControl)
    {
        LOG(ERR) << "Control packets are not supported";
        return 0;
    }

    //check for status messages (no timeout, non blocking)
    {
        size_t chanMask;
        int flags = 0;
        long long timeNs = 0;
        int ret = _device->readStreamStatus(_txStream, chanMask, flags, timeNs, 0);
        *underrun = (ret == SOAPY_SDR_UNDERFLOW);
    }

    //write until length exhausted
    size_t sampsTotal = 0;
    while (sampsTotal < size_t(len))
    {
        int flags = 0;
        long long timeNs = 0;
        const void *txbuffs[bufs.size()];

        //load time on first tx
        if (sampsTotal == 0)
        {
            flags = SOAPY_SDR_HAS_TIME;
            timeNs = this->txTicksToTimeNs(timestamp);
        }

        //load buffer pointers
        for (size_t i = 0; i < bufs.size(); i++) txbuffs[i] = bufs[i] + sampsTotal*2;
        size_t numSampsTx = len-sampsTotal;

        //perform sample write
        int ret = _device->writeStream(_txStream, txbuffs, numSampsTx, flags, timeNs);

        //handle result
        if (ret < 0)
        {
            LOG(ERR) << "writeStream ret=" << ret;
            return 0;
        }
        _numWritten += ret;
        sampsTotal += ret;
    }

    return sampsTotal;
}

bool SoapyDevice::setTxFreq(double wFreq, size_t chan)
{
    return this->setFrequency(SOAPY_SDR_TX, wFreq, chan);
}

bool SoapyDevice::setRxFreq(double wFreq, size_t chan)
{
    return this->setFrequency(SOAPY_SDR_RX, wFreq, chan);
}

bool SoapyDevice::setFrequency(const int dir, double wFreq, size_t chan)
{
    //TODO when LO is shared, we need offset params
    //see _settings.sharedLo

    //enable tuning offset when specified
    SoapySDR::Kwargs kwargs;
    if (_tuningOffset != 0.0)
    {
        LOG(INFO) << "tuning with offset " << (_tuningOffset/1e6) << " MHz";
        char s[64];
        sprintf(s,"%f", _tuningOffset);
        kwargs["OFFSET"] = s;
    }

    _device->setFrequency(SOAPY_SDR_TX, chan, wFreq, kwargs);
    return true;
}

double SoapyDevice::setRxGain(double dB, size_t chan)
{
    _device->setGain(SOAPY_SDR_RX, chan, dB);
    return this->getRxGain(chan);
}

double SoapyDevice::getRxGain(size_t chan)
{
    return _device->getGain(SOAPY_SDR_RX, chan);
}

double SoapyDevice::maxRxGain(void)
{
    return _device->getGainRange(SOAPY_SDR_RX, 0).maximum();
}

double SoapyDevice::minRxGain(void)
{
    return _device->getGainRange(SOAPY_SDR_RX, 0).minimum();
}

double SoapyDevice::setTxGain(double dB, size_t chan)
{
    _device->setGain(SOAPY_SDR_TX, chan, dB);
    return _device->getGain(SOAPY_SDR_TX, chan);
}

double SoapyDevice::maxTxGain(void)
{
    return _device->getGainRange(SOAPY_SDR_TX, 0).maximum();
}

double SoapyDevice::minTxGain(void)
{
    return _device->getGainRange(SOAPY_SDR_TX, 0).minimum();
}

double SoapyDevice::getTxFreq(size_t chan)
{
    return _device->getFrequency(SOAPY_SDR_TX, chan);
}

double SoapyDevice::getRxFreq(size_t chan)
{
    return _device->getFrequency(SOAPY_SDR_RX, chan);
}

double SoapyDevice::getSampleRate()
{
    return _device->getSampleRate(SOAPY_SDR_RX, 0);
}

double SoapyDevice::numberRead()
{
    return _numReadden;
}

double SoapyDevice::numberWritten()
{
    return _numWritten;
}

/***********************************************************************
 * Factory function
 **********************************************************************/
RadioDevice *RadioDevice::make(size_t sps, size_t chans, bool diversity, double offset)
{
    SoapySDR::registerLogHandler(SoapyDevice_logger);
    return new SoapyDevice(sps, chans, diversity, offset);
}
