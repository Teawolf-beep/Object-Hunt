//
// Created by robert on 10.12.19.
//

// System includes
#include <csignal>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <sys/signalfd.h>

// Project includes
#include "Navigator.h"
// Project exceptions
#include "ConnectionManagerException.h"
#include "NavigatorException.h"

volatile int navigation_control_pipe;

// A signal handle to catch the SIGINT and SIGTERM signals
void signalHandler(int signum)
{
    // Check, if we are interested in the caught signal (should be always true)
    if(signum == SIGINT || signum == SIGTERM)
    {
        // Create an exit condition variable with value true
        bool exit_condition = true;
        // Write the exit condition variable to the control pipe of the Navigator object
        if (write(navigation_control_pipe, &exit_condition, sizeof(exit_condition)) == -1)
            fprintf(stderr,"signalHandler(): "
                           "Error while writing to navigation pipe: %s\n", strerror(errno));
    }
    else printf("Unrecognized signal (%X) caught.\n", signum);
}

int main(int argc, char** argv)
{
    struct sigaction sig_act;

    // Initialize the signal handler with the desired signals
    memset(&sig_act, 0, sizeof (sig_act));
    sig_act.sa_handler = signalHandler;
    sigaction(SIGINT, &sig_act, 0);
    sigaction(SIGTERM, &sig_act, 0);
    // Set standard out to line buffering ( needed to log to journalctl)
    setvbuf(stdout, nullptr, _IOLBF, 1024);

    try
    {
        // Create a unique pointer to an object of the application class
        std::unique_ptr<Navigator> navigator = std::make_unique<Navigator>();

        // Get the writing end of the control pipe of the Navigator object
        navigation_control_pipe = navigator->getControlPipe();
        // Run the application (will return upon termination)
        int8_t result = navigator->run();
        // Check the return value
        if (result)
        {
            printf("The navigation process terminated abnormally! Return value: %d\n", result);
            return EXIT_FAILURE;
        }
    }
    // Catch various exceptions and print the error cause
    catch (const ConnectionManagerException& e)
    {
        std::cout << "Connection manager exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const NavigatorException& e)
    {
        std::cout << "Navigator exception caught: " << e.what() << std::endl;
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
        std::cout << "Other exception caught: " << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Leaving main()" << std::endl;
    return EXIT_SUCCESS;
}
