#ifndef REACTOR_CPP_CONNECTION_ENDPOINT_HH
#define REACTOR_CPP_CONNECTION_ENDPOINT_HH

#include "port.hh"

/*
base class for endpoints for federated comms
*/

namespace reactor {

template <class T>
class ConnectionEndpoint {
    Port<T>* port;
};

template <class T>
class UpstreamEndpoint : ConnectionEndpoint<T> {
    
};

template <class T>
class DownstreamEndpoint : ConnectionEndpoint<T> {

};

// reactor ns
}
#endif