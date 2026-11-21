#pragma once
#include <cerrno>

namespace lab26 {
struct CanSendResult {
  int error=0,attempts=0;
  bool ok() const { return error==0; }
};
// The writer returns 0 only for a complete CAN frame, otherwise errno.
template<class Writer,class Wait>
CanSendResult transmitCan(Writer write_frame,Wait wait) {
  for(int attempt=1;attempt<=3;++attempt) {
    const int error=write_frame();
    if(!error)return {0,attempt};
    const bool transient=error==EINTR||error==EAGAIN||error==EWOULDBLOCK||error==ENOBUFS;
    if(!transient||attempt==3)return {error,attempt};
    wait();
  }
  return {EIO,3};
}
}
