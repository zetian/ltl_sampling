#include "config_reader.h"
#include <fstream>
#include <stdlib.h>
#include <string>

int main()
{
    ConfigReader config_reader("test.ini");
    if (config_reader.CheckError()){
        return -1;
    }
    std::cout << config_reader << std::endl;
    std::cout << "get test 1: " << config_reader.Get("test_1", "unknown") << std::endl;
    std::cout << "get test 2: " << config_reader.GetReal("test_21", 0.1) << std::endl;
    std::cout << "get test 3: " << config_reader.GetBoolean("test_3", 0) << std::endl;
    std::cout << "get test 4: " << config_reader.GetInteger("test_4", 0) << std::endl;
}