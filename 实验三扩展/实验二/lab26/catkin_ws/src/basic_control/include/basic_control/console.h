#pragma once
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <cerrno>
#include <sys/ioctl.h>
#include <linux/sockios.h>

// Loopback only: expose through an authenticated SSH tunnel, never on the LAN.
class Console {
  int server=-1, client=-1;
  std::string buffer;
  std::string pending;
  void disconnect(){if(client>=0)close(client);client=-1;connected=false;buffer.clear();pending.clear();}
  void flush(){
    while(client>=0&&!pending.empty()) {
      const auto n=::send(client,pending.data(),pending.size(),MSG_NOSIGNAL|MSG_DONTWAIT);
      if(n>0){pending.erase(0,static_cast<size_t>(n));continue;}
      if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))return;
      disconnect();return;
    }
  }
 public:
  bool connected=false;
  bool canPreview() const {
    int queued=0;
    return client>=0 && pending.empty() && ioctl(client,SIOCOUTQ,&queued)==0 && queued<32768;
  }
  explicit Console(int port=8766) {
    if(port<1024||port>65535)throw std::runtime_error("Invalid console port");
    server=socket(AF_INET,SOCK_STREAM|SOCK_NONBLOCK,0);
    int yes=1; setsockopt(server,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
    sockaddr_in address{}; address.sin_family=AF_INET;
    address.sin_addr.s_addr=htonl(INADDR_LOOPBACK); address.sin_port=htons(port);
    if(server<0||bind(server,(sockaddr*)&address,sizeof(address))<0||listen(server,1)<0)
      throw std::runtime_error("Console port unavailable");
  }
  ~Console() { if(client>=0) close(client); if(server>=0) close(server); }
  template<class Handler> void poll(Handler handler) {
    if(client<0) { client=accept4(server,nullptr,nullptr,SOCK_NONBLOCK); connected=client>=0; }
    if(client<0) return;
    flush();if(client<0)return;
    char data[1024]; auto n=recv(client,data,sizeof(data),0);
    if(n==0 || (n<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK)) { disconnect();return; }
    if(n>0) buffer.append(data,n);
    if(buffer.size()>4096) { disconnect();return; }
    size_t end;
    while((end=buffer.find('\n'))!=std::string::npos) {
      auto command=buffer.substr(0,end);buffer.erase(0,end+1);handler(command);
    }
  }
  void status(const std::string& value) {
    if(client>=0) {
      // Bounded queue: larger point clouds may require multiple nonblocking writes.
      if(pending.size()+value.size()+1>1048576){disconnect();return;}
      pending+=value+'\n';flush();
    }
  }
};
