/*
 * @(#) ProtobufPacket.h   1.0   Feb 25, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_PROTOBUF_PACKET_H_
#define ROBOGEN_PROTOBUF_PACKET_H_

#include <string>
#include <cassert>
#include <vector>
#include <cstdio>
#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>


/**
 * ProtobufPacket implements a simple container
 * (composed of a header and the message payload)
 * for protocol buffers Messages.
 *
 * This class was inspired by public domain code of:
 * Eli Bendersky (eliben@gmail.com).
 */
template<class MessageType>
class ProtobufPacket {

public:

   /**
    * Header size, in bytes
    */
   static const unsigned int HEADER_SIZE = 4;

   /**
    * Empty constructor
    */
   ProtobufPacket() : payload_(new MessageType()) {

   }

   /**
    * Initializes a protobuf packet with the specified message
    */
   ProtobufPacket(boost::shared_ptr<MessageType> message) :
      payload_(message) {
   }

   /**
    * Set the message carried by the packet
    * @param message
    */
   void setMessage(boost::shared_ptr<MessageType> message) {
      this->payload_ = message;
   }

   /**
    * @return the message carried by the packet
    */
   boost::shared_ptr<MessageType> getMessage() {
      return this->payload_;
   }

   /**
    * Forge the packet into the given data buffer (header + message).
    * @return true if the operation completed successful, false otherwise
    */
   bool forge(std::vector<unsigned char>& buf) const {

      if (!this->payload_) {
         return false;
      }

      unsigned messageSize = this->payload_->ByteSize();
      buf.resize(HEADER_SIZE + messageSize);

      // Encode header
      buf[0] = static_cast<boost::uint8_t>((messageSize >> 24) & 0xFF);
      buf[1] = static_cast<boost::uint8_t>((messageSize >> 16) & 0xFF);
      buf[2] = static_cast<boost::uint8_t>((messageSize >> 8) & 0xFF);
      buf[3] = static_cast<boost::uint8_t>(messageSize & 0xFF);

      return this->payload_->SerializeToArray(&buf[HEADER_SIZE], messageSize);
   }

   /**
    * Decode the header of the packet.
    * @return the size of the payload
    */
   unsigned int decodeHeader(const std::vector<unsigned char>& buf) const {

      if (buf.size() < HEADER_SIZE) {
         return 0;
      }

      unsigned messageSize = 0;
      for (unsigned i = 0; i < HEADER_SIZE; ++i) {
         messageSize = messageSize * 256 + (static_cast<unsigned>(buf[i]) & 0xFF);
      }
      return messageSize;
   }

   /**
    * Decode a packet payload.
    * The message payload can be retrieved calling {#getMessage()}
    * @return true if operation completed succesful, false otherwise
    */
   bool decodePayload(const std::vector<unsigned char>& buf) {
      return this->payload_->ParseFromArray(&buf[0], buf.size());
   }


private:

   /**
    * Packet payload
    */
   boost::shared_ptr<MessageType> payload_;
};

#endif /* ROBOGEN_PROTOBUF_PACKET_H_ */
