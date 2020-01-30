//
// Created by robert on 08.01.20.
//

#ifndef OBJECT_HUNT_NAVIGATION_CONNECTIONMANAGER_H
#define OBJECT_HUNT_NAVIGATION_CONNECTIONMANAGER_H

#include <cstdint>
#include <vector>
#include <thread>
#include <mutex>

/**
 * Helps handling multiple socket connections.
 * Administrates socket connections dynamically. Registers new connections and removes closed connections.
 */
class ConnectionManager
{
public:

    /**
     * Custom constructor.
     * Open a socket for further processes and starts a thread to listen for new connections.
     *
     * @throw ConnectionManagerException in case an error occurred while initialization.
     */
    ConnectionManager();

    /**
     * Custom destructor.
     * Joins the child thread and closes active connections.
     */
    ~ConnectionManager();

    /**
     * Gets the file descriptor of active connections.
     *
     * @return A constant reference to the vector containing all active connections.
     */
    const std::vector<int>& getConnections();

    /**
     * Removes a connection.
     * Closes and removes the connection on the passed file descriptor.
     *
     * @param fd The file descriptor that shall be closed.
     */
    void removeConnection(int fd);

    /**
     * Sets the navigation control pipe.
     * Sets the file descriptor to the writing end of the control pipe of the Navigator class. This file descriptor is
     * needed to notify the Navigator class about new connections. The class will only start working, if a valid file
     * descriptor is set. Otherwise no action can be expected.
     *
     * @param fd A valid file descriptor of the writing end of the control pipe.
     */
    void setNavigationControlPipe(int fd);

    /**
     * Connects to the base process.
     *
     * @param base_port The port on which the base process is listening for the navigation process.
     * @return The file descriptor of the connection. A negative value in case something went wrong.
     */
    static int connectToBase(uint16_t base_port);

    /**
     * Closes a socket.
     * Shuts down and closes the socket associated with the passed file descriptor.
     *
     * @param fd A valid file descriptor associated with an active socket connection.
     * @return Zero on success. On error -1 is returned.
     */
    static int closeSocket(int fd);

private:
    static int openSocket(uint16_t port);

    void handleConnections(int server_fd);
    void addConnection(int fd);
    void removeAllConnections();

    int control_pipe[2];
    int navigation_control_pipe;
    std::thread connection_handler;
    std::mutex connection_mutex;
    std::vector<int> active_connections;
};

#endif //OBJECT_HUNT_NAVIGATION_CONNECTIONMANAGER_H
