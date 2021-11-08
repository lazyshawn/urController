#include "../include/sensor_interface.h"

ThreadManager threadManager;

int main(int argc, char** argv) {
  ForceSensor forceSensorLocal;
  Force forceLocal;
  ForceData forceData;

  forceSensorLocal.init_dev();
  for (int i=0; i<90; ++i) {
    std::cout << forceData.forceMat[0][0][0] << "  " 
      << forceData.forceMat[0][0][1] << "  " 
      << forceData.forceMat[0][0][2] << std::endl;
  }

  return 0;
}

