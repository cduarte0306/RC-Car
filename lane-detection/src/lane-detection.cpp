#include "adas.hpp"


using namespace std;

int main()
{
    cout << "Hello CMake. This is my code" << endl;
    ADS* ads = new ADS();
	ads->processDetection();
	return 0;
}
