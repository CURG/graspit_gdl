#include "graspitProtobufServer.h"
#include "Drawable.pb.h"
#include "GraspitMessage.pb.h"
#include "debug.h"

#include "BCI/bciService.h"

GraspitProtobufConnection::GraspitProtobufConnection(QObject *parent, QTcpSocket *socket,
                                                     unsigned int maximum_len) :
    QObject(parent),
    sock(socket),
    maxLen(maximum_len),
    rereadLatency(100)
{
    sock->setReadBufferSize(maxLen);
    connect(sock, SIGNAL(readyRead()),this,SLOT(parseMessage()));
    msg = new GraspitProtobufMessage;
}

GraspitProtobufConnection::~GraspitProtobufConnection()
{
    sock->deleteLater();
    delete msg;
}

void GraspitProtobufConnection::parseMessage()
{
    if (!readMessage()){
      DBGP("GraspitProtobufConnection::parseMessage::Failed to parse message")
      return;
  }
    if(msg->has_drawable_frame())
    {
        DrawableFrame drawableFrame = msg->drawable_frame();
        BCIService::getInstance()->emitDrawableFrame(&drawableFrame);
    }
    if(msg->has_renderable())
    {
        RenderableProtoDrawer().renderMessage(*msg->mutable_renderable());
    }
}

bool GraspitProtobufConnection::readMessage()
{
    // If a retry is pending, don't continue
   if(tryRereadTimer.isActive())
        return false;

   quint32 message_length = getMessageSize(); // The expected message size


   bool readSucceeded = false;


   // Test if there is a valid message in the buffer
   if(message_length && sock->bytesAvailable() >= message_length)
   {
       //Move input cursor forward to beginning of message
       sock->seek(sizeof(message_length));

       // Read the message data and try to parse it.
       QByteArray inputByteArray = sock->read(message_length);
       readSucceeded = msg->ParseFromArray(inputByteArray.data(), inputByteArray.size());

       if(!readSucceeded)
       {
           DBGA("GraspitProtobufServer::Failed to parse message with size "
                << inputByteArray.size());
       }
   }

   int readDelay = readSucceeded ? 1:rereadLatency;
   if(sock->bytesAvailable())
   {
       scheduleReread(readDelay);
   }
   return readSucceeded;
}


void GraspitProtobufConnection::scheduleReread(int readLatency){
   tryRereadTimer.setSingleShot(true);
   tryRereadTimer.setInterval(readLatency);
   connect(&tryRereadTimer, SIGNAL(timeout()), this, SLOT(parseMessage()));
   tryRereadTimer.start();
}

quint32 GraspitProtobufConnection::getMessageSize()
{
    // Try to read the message size prefix
    google::protobuf::uint32 message_length = 0;
    unsigned int prefix_length = sizeof(message_length);
    QByteArray prefix = sock->peek(prefix_length);
    if(prefix_length == prefix.size())
    {

        google::protobuf::io::CodedInputStream::ReadLittleEndian32FromArray(reinterpret_cast<unsigned char *>(prefix.data()),
                                                                            &message_length);
        //If the message size is greater than the read buffer, fail noisily
        if(message_length > maxLen)
        {
            DBGA("GraspitProtobufServer::Message size > socket buffer size: " << message_length);
            return 0;
        }

    }
    return message_length;
}


GraspitProtobufServer::GraspitProtobufServer(unsigned int port_num, QObject * parent ) :
    QTcpServer(parent)
{
    connect(this, SIGNAL(newConnection()), this, SLOT(onConnection()));
    this->listen(QHostAddress::Any,port_num);
}

void GraspitProtobufServer::onConnection()
{

    QTcpSocket *clientQTcpSocketConnection = nextPendingConnection();
    DBGA("GraspitProtobufServer::onConnection:: new connection");
    GraspitProtobufConnection *newGraspitProtobufConnection = new GraspitProtobufConnection(NULL, clientQTcpSocketConnection);
    connect(clientQTcpSocketConnection, SIGNAL(disconnected()),newGraspitProtobufConnection, SLOT(deleteLater()));
}
