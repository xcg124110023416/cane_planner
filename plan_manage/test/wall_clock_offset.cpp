// Test-only ROS wall-time jumps; kernel clocks and thread waits stay untouched.
#include <ros/time.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <cstdlib>
#include <unistd.h>

static int64_t offsetNs() {
  const char* path=std::getenv("CANE_TEST_WALL_OFFSET_FILE");
  if(!path)return 0;
  int fd=open(path,O_RDONLY);
  if(fd<0)return 0;
  char buffer[64];
  ssize_t size=read(fd,buffer,sizeof(buffer)-1);close(fd);
  if(size<=0)return 0;
  buffer[size]=0;
  return std::strtoll(buffer,nullptr,10);
}

namespace ros {
Time Time::now() {
  static auto original=reinterpret_cast<Time(*)()>(dlsym(RTLD_NEXT,"_ZN3ros4Time3nowEv"));
  Time result=original();
  if(Time::isSystemTime())result+=Duration(double(offsetNs())/1e9);
  return result;
}
}
