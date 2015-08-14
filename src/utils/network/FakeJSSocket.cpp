/*
 * FakeJSSocket.cpp
 *
 *  Created on: Aug 11, 2015
 *      Author: guillaume3
 */

#include <utils/network/FakeJSSocket.h>

namespace robogen {

FakeJSSocket::FakeJSSocket() {
	// TODO Auto-generated constructor stub

}
bool FakeJSSocket::open(const std::string& ip, int port) {
	// a fake socket is always opend
	return true;
}

FakeJSSocket::~FakeJSSocket() {
	// TODO Auto-generated destructor stub
}

bool FakeJSSocket::create(int port) {
	// a fake socket is always created
	return true;
}

bool FakeJSSocket::accept() {
	// a fake socket always accept and is not blocking
	return true;
}

bool FakeJSSocket::read(std::vector<unsigned char>& buffer, size_t bytesToRead) {
	// The socket has no data available
	buffer.assign(bytesToRead, 0);
	// but never fail
	return true;
}

bool FakeJSSocket::write(std::vector<unsigned char>& buffer) {
	// we copy the buffer inside the fake socket
	this->innerBuffer = buffer;
	// a fake socket never fails
	return true;
}

bool FakeJSSocket::close() {
	// there is nothing to close so the socket just return
	return true;
}

void FakeJSSocket::interrupt() {
	//there are no blocking calls so the socket just return
	return;
}
std::vector<unsigned char> FakeJSSocket::getContent() {
	return this->innerBuffer;
}
} /* namespace robogen */
