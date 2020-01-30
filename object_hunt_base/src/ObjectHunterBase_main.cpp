//
// Created by robert on 21.11.19.
//

// System includes
#include <memory>
#include <csignal>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <sys/signalfd.h>

// Project includes
#include "TaskDistributor.h"
#include "ObjectHuntDefinitions.h"
// Project exceptions
#include "BNO_055Exception.h"
#include "GearMotorException.h"
#include "GPIOException.h"
#include "HC_SR04Exception.h"
#include "HardwareInterfaceException.h"
#include "I2CException.h"
#include "TaskDistributorException.h"
#include "TCST_1103Exception.h"
#include "VMA_401Exception.h"

int main(int argc, char** argv)
{
    int signal_fd;
    sigset_t signal_set;

    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGTERM);
    sigaddset(&signal_set, SIGINT);

    if (sigprocmask(SIG_BLOCK, &signal_set, nullptr) < 0)
    {
        perror("main() -> sigprocmask");
        return EXIT_FAILURE;
    }

    signal_fd = signalfd(-1, &signal_set, 0);
    if (signal_fd < 0)
    {
        perror("main() -> signalfd");
        return EXIT_FAILURE;
    }

    setvbuf(stdout, nullptr, _IOLBF, 1024);

    try
    {
        std::unique_ptr<TaskDistributor> distributor = std::make_unique<TaskDistributor>(signal_fd);

        int8_t return_value = distributor->run();
        if (return_value < -1) printf("The distributor class terminated abnormally! Return value: %d\n", return_value);
        else if (return_value == ReturnValue::POWER_OFF) printf("Power off pressed!\n");
    }
    catch (const BNO_055Exception& e)
    {
        std::cout << "BNO055 (smart movement sensor) exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const GearMotorException& e)
    {
        std::cout << "Gear motor exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const GPIOException& e)
    {
        std::cout << "Pin: " << (unsigned) e.getPin() << std::endl;
        std::cout << "GPIO exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const HC_SR04Exception& e)
    {
        std::cout << "HC SR04 (ultra sonic sensor) exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const HardwareInterfaceException& e)
    {
        std::cout << "Hardware hw_interface exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const I2CException& e)
    {
        std::cout << "I2C exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const TaskDistributorException& e)
    {
        std::cout << "Task distribution exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const TCST_1103Exception& e)
    {
        std::cout << "TCST 1103 (revolution sensor) exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const VMA_401Exception& e)
    {
        std::cout << "VMA 401 (revolution sensor) exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::system_error& e)
    {
        std::cout << "System error exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch(const std::runtime_error& e)
    {
        std::cout << "Runtime error exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch(const std::exception& e)
    {
        std::cout << "Standard exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (...)
    {
        std::cout << "Other exception caught..." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Leaving main()" << std::endl;
    return EXIT_SUCCESS;
}