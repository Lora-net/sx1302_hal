/**
 * @file Neighbor.h
 * @brief This file is internal to CDP and sorts nearest neighbors
 * on a route path
 * @version
 * @date 2025-7-24
 *
 * @copyright
 */
#ifndef NEIGHBOR_H
#define NEIGHBOR_H
#include "SignalScore.h"
#include <list>

class Neighbor {
    public:
      Neighbor(Duid devId, Duid nextHop, SignalScore signalInfo, unsigned long lastSeen) :
        DeviceId(devId), routingScore(signalInfo.signalScore), lastSeen(lastSeen),
        snr(signalInfo.snr), rssi(signalInfo.rssi),
        freq_hz(signalInfo.freq_hz), sf(signalInfo.sf) {
        // How to handle multiple next hops?
      }
        bool operator>(const Neighbor& other) const {
            return this->routingScore > other.routingScore;
        }
  
      [[nodiscard]] std::string getDeviceId() const { return duckutils::toString(DeviceId); }
      long getRoutingScore() const { return routingScore; }
      unsigned long getLastSeen() const { return lastSeen; }
      long getSnr() { return snr; }
      long getRssi() { return rssi; }
      /// Frequency (Hz) on which this neighbor was last heard. 0 if unknown.
      uint32_t getFreqHz() const { return freq_hz; }
      /// Spreading Factor used by this neighbor. 0 if unknown.
      uint8_t  getSF()     const { return sf; }
  private:
      Duid DeviceId;
      unsigned long lastSeen;
      float snr, rssi, routingScore;
      uint32_t freq_hz = 0;
      uint8_t  sf      = 0;
  };
    
  #endif