#ifndef GRASPITPROTOBUFSERVER_H
#define GRASPITPROTOBUFSERVER_H
#include <QtNetwork>
#include <QTimer>
#include "RenderableProtoDrawer.h"
class GraspitProtobufMessage;
class DrawableFrame;

class GraspitProtobufConnection : public QObject
{
    Q_OBJECT
public:
    GraspitProtobufConnection(QObject * parent, QTcpSocket * socket,
                              unsigned int maximum_len = 38575230);
    ~GraspitProtobufConnection();

signals:
    void updateFrame(DrawableFrame & drawing);

private:
    //! Individual Socket connection
    QTcpSocket *sock;

    //! Maximum size of socket's buffer
    unsigned int maxLen;

    //! Graspit message to read into
    GraspitProtobufMessage *msg;

    //! Timer to reread if more bits are needed or a partial message has been recieved.
    QTimer tryRereadTimer;

    //! Delay at which we should retry parsing the message if not enough data is available.
    int rereadLatency;

    //! Schedule an attempt to parse data in the buffer if necessary.
    void scheduleReread(int readLatency);


    //! Read the message size prefix
    quint32 getMessageSize();

private slots:
    //! Read the message and act on it.
    void parseMessage();

    //! Read the message from the wire and try to parse it.
    bool readMessage();
};

class GraspitProtobufServer : public QTcpServer
{
    Q_OBJECT
public:
    GraspitProtobufServer(unsigned int port_num,QObject * parent = 0);
private slots:
    void onConnection();
};

#endif // GRASPITPROTOBUFSERVER_H
