//
// Copyright (c) 2015-2015 AB Open Ltd
// SPDX-License-Identifier: BSL-1.0
//

#include "radioDevice.h"
#include "Threads.h"
#include "Logger.h"
#include <SoapySDR/Device.hpp>

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
  enum TxWindowType getWindowType();

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

  /** Update the alignment between the read and write timestamps */
  bool updateAlignment(TIMESTAMP timestamp);

  /** Set the transmitter frequency */
  bool setTxFreq(double wFreq, size_t chan);

  /** Set the receiver frequency */
  bool setRxFreq(double wFreq, size_t chan);

  /** Returns the starting write Timestamp*/
  TIMESTAMP initialWriteTimestamp(void);

  /** Returns the starting read Timestamp*/
  TIMESTAMP initialReadTimestamp(void);

  /** returns the full-scale transmit amplitude **/
  double fullScaleInputValue();

  /** returns the full-scale receive amplitude **/
  double fullScaleOutputValue();

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
    SoapySDR::Device *_device;
    SoapySDR::Stream *_rxStream;
    SoapySDR::Stream *_txStream;
    std::vector<size_t> _chans;
    double _sampRate;
};

SoapyDevice::SoapyDevice(size_t sps, size_t chans, bool diversity, double offset):
    _device(NULL),
    _rxStream(NULL),
    _txStream(NULL),
    _sampRate(sps)
{
    for (size_t i = 0; i < chans; i++) _chans.push_back(i);
}

SoapyDevice::~SoapyDevice(void)
{
    if (_rxStream != NULL) _device->closeStream(_rxStream);
    if (_txStream != NULL) _device->closeStream(_txStream);
    SoapySDR::Device::unmake(_device);
}

int SoapyDevice::open(const std::string &args, bool extref)
{
    _device = SoapySDR::Device::make(args);
    if (extref) _device->setClockSource("EXTERNAL");
    _rxStream = _device->setupStream(SOAPY_SDR_RX, "CS16", _chans);
    _txStream = _device->setupStream(SOAPY_SDR_TX, "CS16", _chans);
    for (size_t i = 0; i < _chans.size(); i++)
    {
        _device->setSampleRate(SOAPY_SDR_RX, _chans[i], _sampRate);
        _device->setSampleRate(SOAPY_SDR_TX, _chans[i], _sampRate);
    }
    return NORMAL;
}

RadioDevice *RadioDevice::make(size_t sps, size_t chans, bool diversity, double offset)
{
    return new SoapyDevice(sps, chans, diversity, offset);
}
