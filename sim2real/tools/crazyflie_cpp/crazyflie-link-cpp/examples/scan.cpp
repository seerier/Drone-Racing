#include <iostream>
#include <thread>

#include "crazyflieLinkCpp/Connection.h"

using namespace bitcraze::crazyflieLinkCpp;

int main()
{
    std::cout << "Scanning for Crazyflies..." << std::endl;
    // scanning
    const auto uris = Connection::scan();
    if (uris.empty()) {
        std::cout << "No Crazyflies found." << std::endl;
        return 0;
    }

    for (const auto& uri : uris) {
        std::cout << uri << std::endl;
    }

    return 0;
}