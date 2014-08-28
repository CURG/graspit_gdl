#
# Qt qmake integration with Google Protocol Buffers compiler protoc
#
# To compile protocol buffers with qt qmake, specify PROTOS variable and
# include this file
#
# Example:
# LIBS += /usr/lib/libprotobuf.so
# PROTOS = a.proto b.proto
# include(protobuf.pri)
#
# By default protoc looks for .proto files (including the imported ones) in
# the current directory where protoc is run. If you need to include additional
# paths specify the PROTOPATH variable
#


PROTO_OUTPUT2 = gen_rpcz
VPATH += $$PROTOPATH2 $$PROTO_OUTPUT2
INCLUDEPATH += $$PROTO_OUTPUT2
LIBS += /usr/lib/libprotobuf.so
LIBS += -lboost_thread -lboost_program_options
LIBS += -lzmq

HEADERS += include/Servers/rosRPCZClient.h 

SOURCES += src/Servers/rosRPCZClient.cpp 

LIBS += -L$$PWD/rpcz/build/src/rpcz -lrpcz
LIBS += -L$$PWD/rpcz/build/graspit_rcpz/cpp/ -lgraspit_pb



#####################################
#proto files
for(p, PROTOPATH2):PROTOPATHS2 += --proto_path=$${p}
protobuf_decl2.name = protobuf header
protobuf_decl2.input = PROTOS2
protobuf_decl2.output = ${QMAKE_FILE_BASE}.pb.h
protobuf_decl2.commands = $(MKDIR) $$PROTO_OUTPUT2; protoc --cpp_out=$${PROTO_OUTPUT2} $${PROTOPATHS2} ${QMAKE_FILE_NAME}
protobuf_decl2.variable_out = GENERATED_FILES2
QMAKE_EXTRA_COMPILERS += protobuf_decl2

HEADERS += $$GENERATED_FILES2

protobuf_impl2.name = protobuf implementation
protobuf_impl2.input = PROTOS2
protobuf_impl2.output = $${PROTO_OUTPUT2}/${QMAKE_FILE_BASE}.pb.cc
protobuf_impl2.depends = ${QMAKE_FILE_BASE}.pb.h
protobuf_impl2.commands = $$escape_expand(\\n)
protobuf_impl2.variable_out = GENERATED_SOURCES2
QMAKE_EXTRA_COMPILERS += protobuf_impl2

SOURCES += $$GENERATED_SOURCES2




INCLUDEPATH += rpcz/src rpcz/include

  INCLUDEPATH += rpcz/build/src
  HEADERS += rpcz/build/src/rpcz/rpcz.pb.h

  SOURCES += rpcz/build/src/rpcz/rpcz.pb.cc

  INCLUDEPATH += rpcz/include
  HEADERS += rpcz/service.hpp \
             rpcz/application.hpp\
    rpcz/connection_manager.hpp \
    rpcz/rpc.hpp \
    rpcz/macros.hpp \
    rpcz/callback.hpp \
    rpcz/rpcz.hpp \
    rpcz/sync_event.hpp \
    rpcz/callback.hpp.pump \
    rpcz/rpc_channel.hpp \
    rpcz/server.hpp \
    rpcz/zmq_utils.hpp


HEADERS += rpcz/src/rpcz/rpc_channel_impl.hpp \
        rpcz/src/rpcz/clock.hpp \
        rpcz/src/rpcz/reactor.hpp \
        rpcz/src/rpcz/logging.hpp


SOURCES += rpcz/src/rpcz/application.cc  \
        rpcz/src/rpcz/rpc.cc \
        rpcz/src/rpcz/zmq_utils.cc \
        rpcz/src/rpcz/clock.cc  \
        rpcz/src/rpcz/rpc_channel_impl.cc \
        rpcz/src/rpcz/server.cc \
        rpcz/src/rpcz/reactor.cc \
        rpcz/src/rpcz/connection_manager.cc \
        rpcz/src/rpcz/sync_event.cc
        #rpcz/src/rpcz/zsendrpc.cc


###These should be auto generated

  INCLUDEPATH += rpcz/build/graspit_rcpz/cpp
  HEADERS += rpcz/build/graspit_rcpz/cpp/check_grasp_reachability.pb.h   \
            rpcz/build/graspit_rcpz/cpp/geometry_msgs.pb.h \
            rpcz/build/graspit_rcpz/cpp/get_camera_origin.pb.h\
            rpcz/build/graspit_rcpz/cpp/graspable_object.pb.h\
            rpcz/build/graspit_rcpz/cpp/run_recognition.pb.h\
            rpcz/build/graspit_rcpz/cpp/check_grasp_reachability.rpcz.h\
            rpcz/build/graspit_rcpz/cpp/geometry_msgs.rpcz.h\
            rpcz/build/graspit_rcpz/cpp/get_camera_origin.rpcz.h \
            rpcz/build/graspit_rcpz/cpp/graspable_object.rpcz.h\
            rpcz/build/graspit_rcpz/cpp/run_recognition.rpcz.h\
            rpcz/build/graspit_rcpz/cpp/execute_grasp.rpcz.h\
            rpcz/build/graspit_rcpz/cpp/execute_grasp.pb.h \
            rpcz/build/graspit_rcpz/cpp/grasp.rpcz.h\
            rpcz/build/graspit_rcpz/cpp/grasp.pb.h


  SOURCES +=  rpcz/build/graspit_rcpz/cpp/geometry_msgs.pb.cc \
             rpcz/build/graspit_rcpz/cpp/get_camera_origin.pb.cc \
             rpcz/build/graspit_rcpz/cpp/graspable_object.pb.cc   \
             rpcz/build/graspit_rcpz/cpp/run_recognition.pb.cc\
             rpcz/build/graspit_rcpz/cpp/check_grasp_reachability.rpcz.cc \
             rpcz/build/graspit_rcpz/cpp/check_grasp_reachability.pb.cc \
             rpcz/build/graspit_rcpz/cpp/geometry_msgs.rpcz.cc  \
             rpcz/build/graspit_rcpz/cpp/get_camera_origin.rpcz.cc\
             rpcz/build/graspit_rcpz/cpp/graspable_object.rpcz.cc\
             rpcz/build/graspit_rcpz/cpp/run_recognition.rpcz.cc \
            rpcz/build/graspit_rcpz/cpp/execute_grasp.rpcz.cc\
            rpcz/build/graspit_rcpz/cpp/execute_grasp.pb.cc \
            rpcz/build/graspit_rcpz/cpp/grasp.rpcz.cc\
            rpcz/build/graspit_rcpz/cpp/grasp.pb.cc

############################

    

