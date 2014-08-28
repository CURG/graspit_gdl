#ifndef REQUEST_H
#define REQUEST_H
#include <QObject>
#include "rpcz/rpcz.hpp"
#include "debug.h"

class Request :public QObject
{
    Q_OBJECT
public:
    Request():_rpc(NULL){}

    void sendRequest(QObject * callbackReceiver = NULL, const char * slot = NULL)
    {
        if(_rpc == NULL)
            _rpc = new rpcz::rpc();

        if(_rpc->get_status() !=  rpcz::status::INACTIVE)
            DBGA("ERROR:Incorrect status");
        if(callbackReceiver)
        {
            connect(this, SIGNAL(requestComplete()), callbackReceiver, slot);

        }
           try
           {
               sendRequestImpl();
               std::cout << "REQUEST SENT" << std::endl;
           }
           catch(rpcz::rpc_error &e)
           {
               std::cout << "RPCZ Error: " << e.what() << std::endl;
           }
           catch(const std::exception &e)
           {
               std::cout << "NON RPCZ Error: " << e.what() << std::endl;
           }
    }

    void callback()
    {
        std::cout << "RESPONSE RECEIVED" << std::endl;
        callbackImpl();
        delete _rpc;
        _rpc = new(rpcz::rpc);
        emit requestComplete();
    }

protected:

    virtual void sendRequestImpl() = 0;
    virtual void callbackImpl() = 0;

    rpcz::rpc *_rpc;

signals:

    void requestComplete();

};

#endif // REQUEST_H
