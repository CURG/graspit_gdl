#! /usr/bin/python

import Drawable_pb2
import GraspitMessage_pb2
import sys
import socket

def buildCircleMessage():
  graspitMessage = GraspitMessage_pb2.GraspitProtobufMessage()
  drawable_frame = graspitMessage.drawable_frame
  drawable_frame.frame_number = 1

  circle = drawable_frame.shapes.add();
  circle.id = 1;
  circle.shape_type = Drawable_pb2.ShapeDrawable.Circle
  circle.name = "circle1"
  circle.width = 10
  circle.height = 10
  circle.x = 100
  circle.y = 100
  circle.line_color_argb = 1
  circle.line_width = 2
  circle.fill_color_argb = 1

  return graspitMessage

graspitMessage = buildCircleMessage();

hostname = ('localhost', 4766)

s = socket.socket()
s.connect(hostname)
s.settimeout(5)
s.setblocking(False)


s.send(graspitMessage.SerializeToString())
