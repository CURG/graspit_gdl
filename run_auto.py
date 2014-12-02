#nc localhost 4765
#./bin/graspit -r $GDL_GRASPIT_HAND_PATH -o $GDL_GRASPIT_MODEL_PATH


import os
import subprocess
import time
import socket
from threading import Thread

GDL_MODEL_PATH = os.environ["GDL_MODEL_PATH"]
GDL_HAND_PATH = "/home/jared/grasp_deep_learning/graspit_gdl/models/robots/NewBarrett/NewBarrett.xml"

GDL_GRASPS_PATH = os.environ["GDL_GRASPS_PATH"]

#location for files that are ready for use with gazebo and graspit
PROCESSED_OUTPUT_DIR = GDL_MODEL_PATH +  "/big_bird_models_processed"



def run_subprocess(cmd_string):
	print "running: " + cmd_string

	process = subprocess.Popen(cmd_string.split(), stdout=subprocess.PIPE)
	output = process.communicate()[0]

	print output


def run_loop():

	models = os.listdir(PROCESSED_OUTPUT_DIR)
    
	for model_name in models:
		model_path = PROCESSED_OUTPUT_DIR + '/' + model_name + '/' + model_name + '.xml'

		################################
		# copy in the mtl file
		################################
		cmd = ""
		cmd += './bin/graspit -r ' + GDL_HAND_PATH + ' -o ' + model_path
		#cmd +=  raw_model_dir + "textured_meshes/optimized_tsdf_texture_mapped_mesh.mtl "
		#cmd +=  processed_model_dir
		
		print "Num: %s" % (iterNum)

		t1 = Thread(target=run_subprocess, args=(cmd, ))
		t1.start()

		time.sleep(5)

		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		host = "localhost"
		port = 4765
		sock.connect((host, port))

		sock.send('hello'.encode())

		t1.join()
		#time.sleep(35 * 60)

def move_grasps(iterNum):
	cmd = "mv " + GDL_GRASPS_PATH + "/* " + GDL_GRASPS_PATH + "-loop-" + str(iterNum)
	run_subprocess(cmd)

iterNum = 1

while True:
	run_loop()
	move_grasps(iterNum)

	print "Running again!"
	iterNum = iterNum + 1

