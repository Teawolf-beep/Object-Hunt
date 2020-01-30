//
// Created by robert on 19.11.19.
//

// System includes
#include <cstdio>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <cstring>

// Project includes
#include "GPIO.h"

// Block size to map the GPIO memory
#define BLOCK_SIZE 0x1000000

// Offsets for several registers from the GPIO base address
#define GPSET0 7    //0x1C / 4
#define GPCLR0 10   //0x28 / 4
#define GPLEV0 13   //0x34 / 4
#define GPEDS0 16   //0x40 / 4
#define GPREN0 19   //0x4C / 4
#define GPFEN0 22   //0x58 / 4
#define GPHEN0 25   //0x64 / 4
#define GPLEN0 27   //0x70 / 4
#define GPPUD  37   //0x94 / 4
#define GPPUDCLK0 38 //0x98 / 4

// Sleep time if file access failed
#define SLEEP_TIME 10000
// Max tries for file access
#define MAX_COUNT 500

GPIO::GPIO(const std::string& path)
{
    // Try to open the user accessible GPIO memory
    int mem_fd = open(path.c_str(), O_RDWR | O_SYNC);
    // True, if an error occurred while open the GPIO memory, throw an exception
    if (mem_fd < 0) throw GPIOException("GPIO() -> Error while opening memory: " + std::string(std::strerror(errno)));
    // Map the GPIO memory to any address
    void *map = mmap(nullptr, BLOCK_SIZE, (PROT_READ | PROT_WRITE), MAP_SHARED, mem_fd, 0);
    // No need to keep the file descriptor open after mmap
    close(mem_fd);
    // True, if the mapping of the GPIO memory failed
    if (map == MAP_FAILED)
        throw GPIOException("GPIO() -> Error while mapping memory: " + std::string(std::strerror(errno)));
    // Cast mapped file to volatile pointer, will hold the GPIO base address afterwards
    this->gpio = (volatile uint32_t *) map;
}

GPIO::~GPIO()
{
    // Reset GPIO pins
    printf("~GPIO() called!\n");
    this->releaseAllOutputs();
    this->releaseAllKernelDriver();
}

bool GPIO::isOutput(uint8_t pin)
{
    return std::find(this->outputs.begin(), this->outputs.end(), pin) != this->outputs.end();
}

bool GPIO::isKernelDriverSet(uint8_t pin)
{
    return std::find(this->kernel_driver.begin(),
            this->kernel_driver.end(), pin) != this->kernel_driver.end();
}

bool GPIO::makeInput(uint8_t pin)
{
    // Higher numbers are used internally
    if (pin > 31)
    {
        printf("GPIO::makeInput(), pin %d -> Invalid pin number!\n", pin);
        return false;
    }
    // Check if Pin is already in use as an output
    if (this->isOutput(pin))
    {
        printf("GPIO::makeInput(), pin %d -> Pin is in use as an output!\n", pin);
        return false;
    }
    // Get the address of the respecting function select register
    volatile uint32_t *reg_addr_fsel = gpio + pin / 10;
    // Read the value of the register
    uint32_t value = readReg_(reg_addr_fsel);

    // Set the 3 respecting bits to 0 and write the new value to the register
    value &= ~(7 << ((pin % 10)*3));
    GPIO::writeReg_(reg_addr_fsel, value);
    return true;
}

bool GPIO::makeOutput(uint8_t pin)
{
    // Higher numbers are used internally
    if (pin > 31)
    {
        printf("GPIO::makeOutput(), pin %d -> Invalid pin number!\n", pin);
        return false;
    }
    //Get the address of the respecting function select register
    volatile uint32_t *reg_addr_fsel = gpio + pin / 10;
    //Set the respecting bit for the pin to be an output
    uint32_t value = (1 << ((pin % 10)*3));

    //Set the pin as input first, clears all bits
    this->makeInput(pin);
    //Write the value to the function select register
    GPIO::setBits_(reg_addr_fsel, value);
    // Save the pin number as an output
    this->outputs.push_back(pin);
    // Clear the output (set to low)
    this->clearPin(pin);
    return true;
}

int GPIO::setSingleKernelDriver(uint8_t pin, uint8_t pull_state, const std::string& edge)
{
    // Higher numbers are used internally
    if (pin > 31)
    {
        printf("GPIO::setSingleKernelDriver(). pin %d -> Invalid pin number!\n", pin);
        return -1;
    }
    // Check if Pin is already in use as an output
    if (this->isOutput(pin))
    {
        printf("GPIO::setSingleKernelDriver(), pin %d -> Pin is in use as an output!\n", pin);
        return -2;
    }
    // Get the address of the pull register
    volatile uint32_t *reg_addr_pud = this->gpio + GPPUD;
    // Get the address of the pull clock register
    volatile uint32_t *reg_addr_pudclk = this->gpio + GPPUDCLK0;
    int fd;
    char str[50];
    char buf[8];
    const char dir_in[] = "in";

    // Set the pull state temporarily
    GPIO::writeReg_(reg_addr_pud, (uint32_t) pull_state);

    // Open the file to export GPIO pins to sysfs
    fd = open("/sys/class/gpio/export", O_WRONLY);
    // True, if an error occurred
    if (fd == -1)
    {
        fprintf(stderr,"GPIO::setSingleKernelDriver(), pin %d -> "
                       "Error while opening the GPIO export file: %s\n", pin, strerror(errno));
        return -3;
    }
    // Build the string to export the regarding pin to sysfs
    sprintf(buf, "%d", pin);
    // Write the pin to the export file
    if (write(fd, buf, (pin < 10) ? 1 : 2) == -1)
    {
        fprintf(stderr,"GPIO::setSingleKernelDriver(), pin %d -> "
                       "Error while writing to GPIO export file: %s\n", pin, strerror(errno));
        return -4;
    }
    // Close the export file descriptor
    close(fd);

    // Set pull state of the passed pin
    uint32_t value = 1 << pin;
    // Write the pin to the pull clock register
    GPIO::writeReg_(reg_addr_pudclk, value);

    // Build the string to open the file to set the direction (input/output) of the pin
    sprintf(str, "/sys/class/gpio/gpio%d/direction", pin);
    // The udev rule needs time to change the permission of GPIO file, we try to access it in a loop
    do
    {
        uint32_t counter = 0;
        fd = open(str, O_WRONLY);
        // True, if we could not open the file (most likely because the permission is not changed)
        if (fd == -1)
        {
            // Wait ten milliseconds
            usleep(SLEEP_TIME);
            // Increment and check the counter
            if (++counter > MAX_COUNT)
            {
                fprintf(stderr,"GPIO::setSingleKernelDriver(), pin %d -> "
                               "Error while opening GPIO direction file: %s\n", pin, strerror(errno));
                return -5;
            }
        }
    } while (fd == -1);
    // Write in (for input) to direction file
    if (write(fd, dir_in, 2) == -1)
    {
        fprintf(stderr,"GPIO::setSingleKernelDriver(), pin %d -> "
                       "Error while writing to GPIO direction file: %s\n", pin, strerror(errno));
        return -6;
    }
    // Close direction file
    close(fd);

    // Build the string to open the file to set the edge of the pin
    sprintf(str, "/sys/class/gpio/gpio%d/edge", pin);
    // The udev rule needs time to change the permission of GPIO file, we try to access it in a loop again
    do
    {
        uint32_t counter = 0;
        fd = open(str, O_WRONLY);
        // True, if we could not open the file (most likely because the permission is not changed)
        if (fd == -1)
        {
            // Wait ten milliseconds
            usleep(SLEEP_TIME);
            // Increment and check the counter
            if (++counter > MAX_COUNT)
            {
                fprintf(stderr,"GPIO::setSingleKernelDriver(), pin %d -> "
                               "Error while opening GPIO edge file: %s\n", pin, strerror(errno));
                return -7;
            }
        }
    } while (fd == -1);
    // Write the passed edge to the edge file
    if (write(fd, edge.c_str(), edge.length()) == -1)
    {
        fprintf(stderr,"GPIO::setSingleKernelDriver(), pin %d -> "
                       "Error while writing to GPIO edge file: %s\n", pin, strerror(errno));
        return -8;
    }
    // Build the string to read the pin value
    sprintf(str, "/sys/class/gpio/gpio%d/value", pin);
    // Open the file (No loop needed since we open in read only mode
    fd = open(str, O_RDONLY);
    // True, if an error occurred
    if (fd < 0)
    {
        fprintf(stderr,"GPIO::setSingleKernelDriver(), pin %d -> "
                       "Error while opening to GPIO value file: %s\n", pin, strerror(errno));
        return -9;
    }
    // Consume former interrupts
    lseek(fd, 0, SEEK_SET);
    // Read the file to clear pending interrupts
    read(fd, buf, sizeof buf);

    // Reset the pull state register
    GPIO::writeReg_(reg_addr_pud, 0);
    // Reset the pull clock register
    GPIO::writeReg_(reg_addr_pudclk, 0);

    // Save the pin number
    this->kernel_driver.push_back(pin);

    // Return the opened file descriptor
    return fd;
}

bool GPIO::setMultipleKernelDriver(std::vector<InterruptDescriptor> &descriptors, uint8_t pull_state)
{
    // Check all passed pins
    for (const InterruptDescriptor& descriptor : descriptors)
    {
        // Higher numbers are used internally
        if (descriptor.pin > 31)
        {
            printf("GPIO::setMultipleKernelDriver(), pin %d -> Invalid pin number!\n", descriptor.pin);
            return false;
        }
        // Check if Pin is already in use as an output
        if (this->isOutput(descriptor.pin))
        {
            printf("GPIO::setMultipleKernelDriver(), pin %d -> "
                   "Pin is in use as an output!\n", descriptor.pin);
            return false;
        }
    }
    // Get the address of the pull register
    volatile uint32_t *reg_addr_pud = this->gpio + GPPUD;
    // Get the address of the pull clock register
    volatile uint32_t *reg_addr_pudclk = this->gpio + GPPUDCLK0;
    int fd;
    char str[50];
    char buf[8];
    const char dir_in[] = "in";

    // Set the pull state temporarily
    GPIO::writeReg_(reg_addr_pud, (uint32_t) pull_state);

    // Open the file to export GPIO pins to sysfs
    fd = open("/sys/class/gpio/export", O_WRONLY);
    // True, if an error occurred
    if (fd == -1)
    {
        fprintf(stderr,"GPIO::setMultipleKernelDriver() -> "
                       "Error while opening the GPIO export file: %s\n", strerror(errno));
        return false;
    }
    // Export all passed pins to sysfs in a loop
    for (const InterruptDescriptor& descriptor : descriptors)
    {
        // Build the string to export the regarding pin to sysfs
        sprintf(buf, "%d", descriptor.pin);
        // Write the pin to the export file
        if (write(fd, buf, (descriptor.pin < 10) ? 1 : 2) == -1)
        {
            fprintf(stderr,"GPIO::setMultipleKernelDriver() -> "
                           "Error while writing to GPIO export file: %s\n", strerror(errno));
            return false;
        }
    }
    // Close the export file descriptor
    close(fd);

    // Set pull state, direction, edge and read the value of all passed pins in a loop
    for (InterruptDescriptor& descriptor : descriptors)
    {
        // Set pull state of the passed pin
        uint32_t value = 1 << descriptor.pin;
        // Write the pin to the pull clock register
        GPIO::writeReg_(reg_addr_pudclk, value);

        // Build the string to open the file to set the direction (input/output) of the pin
        sprintf(str, "/sys/class/gpio/gpio%d/direction", descriptor.pin);
        // The udev rule needs time to change the permission of GPIO file, we try to access it in a loop
        do
        {
            uint32_t counter = 0;
            fd = open(str, O_WRONLY);
            // True, if we could not open the file (most likely because the permission is not changed)
            if (fd == -1)
            {
                // Wait ten milliseconds
                usleep(SLEEP_TIME);
                // Increment and check the counter
                if (++counter > MAX_COUNT)
                {
                    fprintf(stderr,"GPIO::setMultipleKernelDriver(), pin %d -> "
                                   "Error while opening to GPIO value file: %s\n", descriptor.pin, strerror(errno));
                    return false;
                }
            }
        } while (fd == -1);
        // Write in (for input) to direction file
        if (write(fd, dir_in, 2) == -1)
        {
            fprintf(stderr,"GPIO::setMultipleKernelDriver(), pin %d -> "
                           "Error while writing to GPIO direction file: %s\n", descriptor.pin, strerror(errno));
            return false;
        }
        // Close direction file
        close(fd);

        // Build the string to open the file to set the edge of the pin
        sprintf(str, "/sys/class/gpio/gpio%d/edge", descriptor.pin);
        // The udev rule needs time to change the permission of GPIO file, we try to access it in a loop again
        do
        {
            uint32_t counter = 0;
            fd = open(str, O_WRONLY);
            // True, if we could not open the file (most likely because the permission is not changed)
            if (fd == -1)
            {
                // Wait ten milliseconds
                usleep(SLEEP_TIME);
                // Increment and check the counter
                if (++counter > MAX_COUNT)
                {
                    fprintf(stderr,"GPIO::setMultipleKernelDriver(), pin %d -> "
                                   "Error while opening GPIO edge file: %s\n", descriptor.pin, strerror(errno));
                    return false;
                }
            }
        } while (fd == -1);
        // Write the passed edge to the edge file
        if (write(fd, descriptor.edge.c_str(), descriptor.edge.length()) == -1)
        {
            fprintf(stderr,"GPIO::setMultipleKernelDriver(), pin %d -> "
                           "Error while writing to GPIO edge file: %s\n", descriptor.pin, strerror(errno));
            return false;
        }
        // Build the string to read the pin value
        sprintf(str, "/sys/class/gpio/gpio%d/value", descriptor.pin);
        // Open the file (No loop needed since we open in read only mode
        descriptor.fd = open(str, O_RDONLY);
        // True, if an error occurred
        if (descriptor.fd < 0)
        {
            fprintf(stderr,"GPIO::setMultipleKernelDriver(), pin %d -> "
                           "Error while opening GPIO value file: %s\n", descriptor.pin, strerror(errno));
            return false;
        }
        // Consume former interrupts
        lseek(descriptor.fd, 0, SEEK_SET);
        // Read the file to clear pending interrupts
        read(descriptor.fd, buf, sizeof buf);

        // Save the pin number
        this->kernel_driver.push_back(descriptor.pin);
    }
    // Reset the pull state register
    GPIO::writeReg_(reg_addr_pud, 0);
    // Reset the pull clock register
    GPIO::writeReg_(reg_addr_pudclk, 0);

    return true;
}

bool GPIO::releaseSingleKernelDriver(uint8_t pin)
{
    if (!this->isKernelDriverSet(pin))
    {
        printf("GPIO::releaseSingleKernelDriver(), pin %d -> "
               "Cannot remove kernel driver since no kernel driver is associated with this pin\n", pin);
        return false;
    }
    char buf[2];

    // Open the file to unexport GPIO numbers
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    // True, if an error occurred
    if (fd == -1)
    {
        fprintf(stderr, "GPIO::releaseSingleKernelDriver(), pin %d -> "
                        "Error while opening GPIO value file: %s\n", pin, strerror(errno));
        return false;
    }
    // Build the string to unexport the pin
    sprintf(buf, "%d", pin);
    // Write the pin number to the unexport file
    if (write(fd, buf, (pin < 10) ? 1 : 2) == -1)
    {
        fprintf(stderr, "GPIO::releaseSingleKernelDriver(), pin %d -> "
                        "Error while writing to GPIO unexport file: %s\n", pin, strerror(errno));
        return false;
    }
    // Remove the passed pin number from the kernel driver vector
    this->kernel_driver.erase(std::remove(this->kernel_driver.begin(),
            this->kernel_driver.end(), pin), this->kernel_driver.end());
    return true;
}

void GPIO::releaseAllKernelDriver()
{
    char buf[2];

    // Open the file to unexport GPIO numbers
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    // True, if an error occurred
    if (fd == -1)
    {
        printf("GPIO::releaseMultipleKernelDriver() -> "
               "Error while opening GPIO unexport file: %s\n", strerror(errno));
        return;
    }
    for (uint8_t pin : this->kernel_driver)
    {
        if (!this->isKernelDriverSet(pin)) continue;
        // Build the string to unexport the pin
        sprintf(buf, "%d", pin);
        // Write the pin number to the unexport file
        if (write(fd, buf, (pin < 10) ? 1 : 2) == -1)
        {
            printf("GPIO::releaseMultipleKernelDriver() -> "
                   "Error while writing to GPIO unexport file: %s\n", strerror(errno));
            return;
        }
    }
    this->kernel_driver.clear();
}

bool GPIO::setPullState(uint8_t pin, uint8_t state)
{
    // Higher numbers are used internally
    if (pin > 31)
    {
        printf("GPIO::setPullState(), pin %d -> Invalid pin number!\n", pin);
        return false;
    }
    // Get the address of the pull register
    volatile uint32_t *reg_addr_pud = this->gpio + GPPUD;
    // Get the address of the pull clock register
    volatile uint32_t *reg_addr_pudclk = this->gpio + GPPUDCLK0;

    // Set pull state of all pins through write to pull register
    GPIO::writeReg_(reg_addr_pud, (uint32_t) state);
    // Wait (it is advised to wait 150 cycles in the BCM data sheet)
    usleep(10);

    // Build value
    uint32_t value = 1 << pin;
    // Write value
    GPIO::writeReg_(reg_addr_pudclk, value);

    // Wait again (advised in the BCM data sheet)
    usleep(10);
    // Reset the register values
    GPIO::writeReg_(reg_addr_pud, 0);
    GPIO::writeReg_(reg_addr_pudclk, 0);
    return true;
}

void GPIO::setPullState(const std::vector<uint8_t>& pins, uint8_t state)
{
    // Get the address of the pull register
    volatile uint32_t *reg_addr_pud = this->gpio + GPPUD;
    // Get the address of the pull clock register
    volatile uint32_t *reg_addr_pudclk = this->gpio + GPPUDCLK0;
    uint32_t value;

    // Set pull state of all pins through write to pull register
    GPIO::writeReg_(reg_addr_pud, (uint32_t) state);
    // Wait (it is advised to wait 150 cycles in the BCM data sheet)
    usleep(10);
    // Pull state stays permanent through write to pull clock register
    for (uint8_t pin : pins)
    {
        // Higher numbers are used internally
        if (pin > 31) continue;
        value = 1 << pin;
        GPIO::writeReg_(reg_addr_pudclk, value);
    }
    // Wait again (advised in the BCM data sheet)
    usleep(10);
    // Reset the register values
    GPIO::writeReg_(reg_addr_pud, 0);
    GPIO::writeReg_(reg_addr_pudclk, 0);
}

bool GPIO::readPin(uint8_t pin)
{
    // Get address of the level register
    volatile uint32_t *reg_addr_lev = this->gpio + GPLEV0;
    // Read register value
    uint32_t value = GPIO::readReg_(reg_addr_lev);

    // Return the regarding bit value
    return (value & (1 << pin));
}

bool GPIO::setPin(uint8_t pin)
{
    // Higher numbers are used internally
    if (pin > 31)
    {
        printf("GPIO::setPin(). pin %d -> Invalid pin number!\n", pin);
        return false;
    }
    // Check if Pin is set as an output
    if (!this->isOutput(pin))
    {
        printf("GPIO::setPin(), pin %d -> Pin is not set as an output!\n", pin);
        return false;
    }
    // Get address of the set register
    volatile uint32_t *reg_addr_set = this->gpio + GPSET0;
    // Write a 1 to the respecting position
    uint32_t value = 1 << pin;

    // Write the value to the set register
    GPIO::writeReg_(reg_addr_set, value);
    return true;
}

bool GPIO::clearPin(uint8_t pin)
{
    // Higher numbers are used internally
    if (pin > 31)
    {
        printf("GPIO::clearPin(). pin %d -> Invalid pin number!\n", pin);
        return false;
    }
    // Check if Pin is set as an output
    if (!this->isOutput(pin))
    {
        printf("GPIO::clearPin(), pin %d -> Pin is not set as an output!\n", pin);
        return false;
    }
    // Get the address of the clear register
    volatile uint32_t *reg_addr_clr = this->gpio + GPCLR0;
    // Write a 1 to the respecting position
    uint32_t value = 1 << pin;

    // Write the value to the clear register
    GPIO::writeReg_(reg_addr_clr, value);
    return true;
}

bool GPIO::releaseOutput(uint8_t pin)
{
    // Higher numbers are used internally
    if (pin > 31)
    {
        printf("GPIO::releaseOutput(). pin %d -> Invalid pin number!\n", pin);
        return false;
    }
    // Check if Pin is set as an output
    if (!this->isOutput(pin))
    {
        printf("GPIO::releaseOutput(), pin %d -> Pin is not set as an output!\n", pin);
        return false;
    }
    // Make this pin an input
    this->resetPin_(pin);

    // Remove this pin from the output list
    this->outputs.erase(std::remove(this->outputs.begin(),
                                     this->outputs.end(), pin), this->outputs.end());
    return true;
}

void GPIO::releaseAllOutputs()
{
    // Make all outputs to inputs
    for (uint8_t output : this->outputs) this->resetPin_(output);
    // Clear the outputs vector
    this->outputs.clear();
}

void GPIO::clearOutputs()
{
    for (uint8_t pin : this->outputs) this->clearPinNoCheck_(pin);
}

////////////////////////////////////////////////////////////////////////////////////////
// Private static method to write a value to a register. Expects the register address and the
// value to be written to the register. It will always be written the whole register.
////////////////////////////////////////////////////////////////////////////////////////

void GPIO::writeReg_(volatile uint32_t *addr, uint32_t value)
{
    // Creates a memory barrier and writes the register
    __sync_synchronize();
    *addr = value;
    __sync_synchronize();
}

////////////////////////////////////////////////////////////////////////////////////////
// Private static method to read from a register. Expects the address of the register that
// should be read. It will always be read the whole register.
////////////////////////////////////////////////////////////////////////////////////////

uint32_t GPIO::readReg_(const volatile uint32_t *addr)
{
    // Creates a memory barrier and reads the register
    __sync_synchronize();
    uint32_t return_value = *addr;
    __sync_synchronize();
    return return_value;
}

////////////////////////////////////////////////////////////////////////////////////////
// Private static method to set individual bits in one register. Expects the register address
// and the bits to be written to the register.
////////////////////////////////////////////////////////////////////////////////////////

void GPIO::setBits_(volatile uint32_t *addr, uint32_t value)
{
    uint32_t buf = readReg_(addr);
    buf |= value;
    GPIO::writeReg_(addr, buf);
}

////////////////////////////////////////////////////////////////////////////////////////
// Private member method that does basically the same as makInput() without checking the passed value.
////////////////////////////////////////////////////////////////////////////////////////

void GPIO::resetPin_(uint8_t pin)
{
    // Get the address of the respecting function select register
    volatile uint32_t *reg_addr_fsel = gpio + pin / 10;
    // Read the value of the register
    uint32_t value = readReg_(reg_addr_fsel);

    // Set the 3 respecting bits to 0 and write the new value to the register
    value &= ~(7 << ((pin % 10)*3));
    GPIO::writeReg_(reg_addr_fsel, value);
}

////////////////////////////////////////////////////////////////////////////////////////
// Private member method that does basically the same as clearPin() without checking the passed value.
////////////////////////////////////////////////////////////////////////////////////////

void GPIO::clearPinNoCheck_(uint8_t pin)
{
    // Get the address of the clear register
    volatile uint32_t *reg_addr_clr = this->gpio + GPCLR0;
    // Write a 1 to the respecting position
    uint32_t value = 1 << pin;

    // Write the value to the clear register
    GPIO::writeReg_(reg_addr_clr, value);
}
