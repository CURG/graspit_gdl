# Linux-specific libraries for GraspIt!. Included from graspit.pro - not for standalone use.

# ---------------------- Blas and Lapack ----------------------------------
LIBS=
LIBS += -lblas -llapack 

HEADERS += include/lapack_wrappers.h


# ---------------------- General libraries and utilities ----------------------------------

LIBS	+= $$PWD/qhull/libqhull.a -L$(COINDIR)/lib -lSoQt -lCoin -lGL -lpthread

MOC_DIR = .moc
OBJECTS_DIR = .obj

#------------------------------------ add-ons --------------------------------------------

INCLUDEPATH += /usr/include/pcl-1.7/
INCLUDEPATH += /usr/local/include/eigen3/

LIBS += -lboost_filesystem -lboost_system \
        -lpcl_registration -lpcl_sample_consensus -lpcl_features -lpcl_filters -lpcl_surface -lpcl_segmentation \
        -lpcl_search -lpcl_kdtree -lpcl_octree -lflann_cpp -lpcl_common -lpcl_io -lpcl_visualization

mosek {
	error("Mosek linking only tested under Windows")
}

boost {
	error("Boost linking only tested under Windows")
}

hardwarelib {
	error("Hardware library only available under Windows")
}
