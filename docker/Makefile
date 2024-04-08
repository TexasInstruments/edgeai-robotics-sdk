#!make
MAKEFLAGS += --no-print-directory

# Defaults. These can be changed from command line
ROS_VER          ?= 2

# GPUS: if 'y', '--gpus all' argument is added in 'docker run' command
GPUS             ?= n

# Robotics SDK version
SDK_VER          = 9.2.0
EDGEAI_VER       = 9.2.0.5
TIVA_LIB_VER     ?= 9.2.0
RPMSG_LIB_VER    ?= 0.6.6

# Set the operating variables based on the ROS version specified
ROS_DISTRO        = humble
WORK_DIR          = j7ros_home
REPO_NAME         = robotics_sdk
WS_NAME           = ros_ws
ROS_TAG           = ros2
EXCLUDE           = ros1

ifeq ($(ROS_VER), 1)
	ROS_DISTRO    = noetic
	ROS_TAG       = ros1
	EXCLUDE       = ros2
endif

# set PLATFORM based on "uname -m"
PLATFORM          = j7
ARCH              = arm64v8
ifeq ($(shell uname -m), x86_64)
	PLATFORM      = pc
	ARCH          = x86_64
endif

WORK_PATH_HOST    = $(HOME)/$(WORK_DIR)
REPO_PATH         = /opt/$(REPO_NAME)
ifeq ($(PLATFORM), pc)
	REPO_PATH     = $(WORK_PATH_HOST)/ros_ws/src/$(REPO_NAME)
endif
ROS_SRC_DIR       = $(REPO_PATH)
DATA_DIR          = $(WORK_PATH_HOST)/data
DOCKER_DIR        = $(ROS_SRC_DIR)/docker
SCRIPTS_DIR       = $(ROS_SRC_DIR)/docker/scripts
DLRT_DIR          = /opt/dl_runtime

ifeq ($(ROS_DOCKER_INTERNAL),)
	USE_PROXY = 0
else
	USE_PROXY = 1
endif

# Environment variable list to pass to the Docker container
ENV_FILE          = env_list.txt
ENV_FILE_PATH     = $(DOCKER_DIR)/$(ENV_FILE)

# Tarball filenames
ROSBAG_ZED_FILE       = ros2-bag_zed1_2020_1109.tar.gz
ROSBAG_6DPOSE_FILE    = ros2-bag_pose_6d_nv12.tar.gz
ROSBAG_HUMANPOSE_FILE = ros2-bag_human_pose_nv12.tar.gz
ROSBAG_ODRADAR_FILE   = camera_radar_2024_03_18-18_58_23.tar.gz
VISLOC_DATA_FILE      = visual-localization-data-ros2_2022_0908.tar.gz
LIDAR2D_FILE          = lidar2d_2021_0521.tar.gz
CALIB_IMGS_FILE       = calib-images-example_2021_0422.tar.gz

# Tarball URLs
URL_BASE              = https://software-dl.ti.com/jacinto7/esd/robotics-sdk/data
ROSBAG_ZED_URL        = $(URL_BASE)/$(ROSBAG_ZED_FILE)
ROSBAG_6DPOSE_URL     = $(URL_BASE)/$(ROSBAG_6DPOSE_FILE)
ROSBAG_HUMANPOSE_URL  = $(URL_BASE)/$(ROSBAG_HUMANPOSE_FILE)
ROSBAG_ODRADAR_URL    = $(URL_BASE)/$(ROSBAG_ODRADAR_FILE)
VISLOC_DATA_URL       = $(URL_BASE)/$(VISLOC_DATA_FILE)
LIDAR2D_LOC_URL       = $(URL_BASE)/$(LIDAR2D_FILE)
CALIB_IMGS_URL        = $(URL_BASE)/$(CALIB_IMGS_FILE)

# IP address
J7_IP_ADDR       := $(shell ifconfig | grep -A 1 'eth0' | tail -1 | awk '{print $$2}')

## Set up environments on J7 host linux
opkg_config:
	$(SCRIPTS_DIR)/config_opkg_repo.sh

ipk_install:
	$(SCRIPTS_DIR)/install_tivision_apps_ipk.sh

ip_show:
	@echo "$(J7_IP_ADDR)"

info:
	$(info PLATFORM          = $(PLATFORM))
	$(info WORK_DIR          = $(WORK_DIR))
	$(info REPO_NAME         = $(REPO_NAME))
	$(info WORK_PATH_HOST    = $(WORK_PATH_HOST))
	$(info REPO_PATH         = $(REPO_PATH))
	$(info ROS_SRC_DIR       = $(ROS_SRC_DIR))
	$(info DATA_DIR          = $(DATA_DIR))
	$(info DOCKER_DIR        = $(DOCKER_DIR))
	$(info SCRIPTS_DIR       = $(SCRIPTS_DIR))

## Docker
# generate "docker build" script
docker_build:
	@echo "#!/bin/bash"                                       >  $(TGT_NAME)
	@echo "set -e"                                            >> $(TGT_NAME)
	@echo "SECONDS=0"                                         >> $(TGT_NAME)
	@echo "ROS_DISTRO=$(ROS_DISTRO)"                          >> $(TGT_NAME)
	@echo "SDK_VER=$(SDK_VER)"                                >> $(TGT_NAME)
	@echo "DOCKER_TAG=$(PLATFORM)-ros-\$$ROS_DISTRO-common:\$$SDK_VER" >> $(TGT_NAME)
ifeq ($(PLATFORM), j7)
	@echo "DOCKER_TAG2=$(PLATFORM)-ros-\$$ROS_DISTRO-\$$SOC:\$$SDK_VER" >> $(TGT_NAME)
endif
	@echo "DOCKER_DIR=$(DOCKER_DIR)"                          >> $(TGT_NAME)
	@echo ": \"\$${USE_PROXY:=$(USE_PROXY)}\""                >> $(TGT_NAME)
	@echo "# modify the server and proxy URLs as requied"     >> $(TGT_NAME)
	@echo "if [ \"\$${USE_PROXY}\" -ne \"0\" ]; then"         >> $(TGT_NAME)
	@echo "    REPO_LOCATION=$(ROS_DOCKER_INTERNAL)"          >> $(TGT_NAME)
	@echo "    HTTP_PROXY=${HTTP_PROXY}"                      >> $(TGT_NAME)
	@echo "else"                                              >> $(TGT_NAME)
	@echo "    REPO_LOCATION="                                >> $(TGT_NAME)
	@echo "fi"                                                >> $(TGT_NAME)
	@echo "echo \"USE_PROXY = \$$USE_PROXY\""                 >> $(TGT_NAME)
	@echo "echo \"REPO_LOCATION = \$$REPO_LOCATION\""         >> $(TGT_NAME)
	@echo "DST_DIR=$(WORK_PATH_HOST)/docker_src"              >> $(TGT_NAME)
	@echo "EDGEAI_VER=$(EDGEAI_VER)"                          >> $(TGT_NAME)
	@echo "TIVA_LIB_VER=$(TIVA_LIB_VER)"                      >> $(TGT_NAME)
	@echo "RPMSG_LIB_VER=$(RPMSG_LIB_VER)"                    >> $(TGT_NAME)
	@echo "bash \$$DOCKER_DIR/scripts/prepare_docker_build.sh \$$DST_DIR \$$TIVA_LIB_VER \$$RPMSG_LIB_VER" >> $(TGT_NAME)
	@echo "cd $(WORK_PATH_HOST)"                              >> $(TGT_NAME)
	@echo "if [ \"\$$(docker images -q \$$DOCKER_TAG 2> /dev/null)\" == \"\" ]; then" >> $(TGT_NAME)
	@echo "    docker build \\"                                   >> $(TGT_NAME)
	@echo "        -t \$$DOCKER_TAG \\"                           >> $(TGT_NAME)
	@echo "        --build-arg USE_PROXY=\$$USE_PROXY \\"         >> $(TGT_NAME)
	@echo "        --build-arg REPO_LOCATION=\$$REPO_LOCATION \\" >> $(TGT_NAME)
	@echo "        --build-arg HTTP_PROXY=\$$HTTP_PROXY \\"       >> $(TGT_NAME)
ifeq ($(PLATFORM), pc)
	@echo "        --build-arg EDGEAI_VER=\$$EDGEAI_VER \\"       >> $(TGT_NAME)
endif
	@echo "        -f \$$DOCKER_DIR/Dockerfile.$(ARCH).\$$ROS_DISTRO ./docker_src"  >> $(TGT_NAME)
	@echo "    echo \"Docker build -t \$$DOCKER_TAG completed!\"" >> $(TGT_NAME)
	@echo "else"                                              >> $(TGT_NAME)
	@echo "    echo \"\$$DOCKER_TAG already exists.\""        >> $(TGT_NAME)
	@echo "fi"                                                >> $(TGT_NAME)
ifeq ($(PLATFORM), j7)
	@echo "docker build \\"                                   >> $(TGT_NAME)
	@echo "    -t \$$DOCKER_TAG2 \\"                          >> $(TGT_NAME)
	@echo "    --build-arg USE_PROXY=\$$USE_PROXY \\"         >> $(TGT_NAME)
	@echo "    --build-arg HTTP_PROXY=\$$HTTP_PROXY \\"       >> $(TGT_NAME)
	@echo "    --build-arg TIVA_LIB_VER=\$$TIVA_LIB_VER \\"   >> $(TGT_NAME)
	@echo "    --build-arg RPMSG_LIB_VER=\$$RPMSG_LIB_VER \\" >> $(TGT_NAME)
	@echo "    --build-arg SOC_NAME=\$$SOC \\"                >> $(TGT_NAME)
	@echo "    --build-arg ROS_DISTRO=\$$ROS_DISTRO \\"       >> $(TGT_NAME)
	@echo "    --build-arg SDK_VER=\$$SDK_VER \\"             >> $(TGT_NAME)
	@echo "    --build-arg EDGEAI_VER=\$$EDGEAI_VER \\"       >> $(TGT_NAME)
	@echo "    -f \$$DOCKER_DIR/Dockerfile.$(ARCH).ti_libs ./docker_src"  >> $(TGT_NAME)
	@echo "echo \"Docker build -t \$$DOCKER_TAG2 completed!\"" >> $(TGT_NAME)
endif
	@echo "duration=\$$SECONDS"                               >> $(TGT_NAME)
	@echo "echo \"\$$((\$$duration / 60)) minutes and \$$((\$$duration % 60)) seconds elapsed.\"" >> $(TGT_NAME)
	@echo "rm -r \$$DST_DIR"	                              >> $(TGT_NAME)
	@chmod +x $(TGT_NAME)
	@echo "$(TGT_NAME) is generated"

# generate "docker run" script
docker_run:
	@echo "#!/bin/bash"                                       >  $(TGT_NAME)
	@echo "ROS_DISTRO=$(ROS_DISTRO)"                          >> $(TGT_NAME)
	@echo "SDK_VER=$(SDK_VER)"                                >> $(TGT_NAME)
ifeq ($(PLATFORM), j7)
	@echo "DOCKER_TAG=$(PLATFORM)-ros-\$$ROS_DISTRO-\$$SOC:\$$SDK_VER" >> $(TGT_NAME)
endif
ifeq ($(PLATFORM), pc)
	@echo "DOCKER_TAG=$(PLATFORM)-ros-\$$ROS_DISTRO-common:\$$SDK_VER" >> $(TGT_NAME)
endif
	@echo ": \"\$${USE_PROXY:=$(USE_PROXY)}\""                >> $(TGT_NAME)
ifeq ($(PLATFORM), j7)
	@echo "DOCKER_DIR=$(DOCKER_DIR)"                          >> $(TGT_NAME)
ifeq ($(ROS_VER), 1)
	@echo "IP_ADDR=\$$(ifconfig | grep -A 1 'eth0' | tail -1 | awk '{print \$$2}')"       >> $(TGT_NAME)
	@echo "if [[ ! \$$IP_ADDR =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+\$$ ]]; then"             >> $(TGT_NAME)
	@echo "    IP_ADDR=\$$(ifconfig | grep -A 1 'wlp1s0' | tail -1 | awk '{print \$$2}')" >> $(TGT_NAME)
	@echo "fi"                                                >> $(TGT_NAME)
endif
endif
	@echo "if [ \"\$$#\" -lt 1 ]; then"                       >> $(TGT_NAME)
	@echo "    CMD=/bin/bash"                                 >> $(TGT_NAME)
	@echo "else"                                              >> $(TGT_NAME)
	@echo "    CMD=\"\$$@\""                                  >> $(TGT_NAME)
	@echo "fi"                                                >> $(TGT_NAME)
ifeq ($(PLATFORM), pc)
	@echo "xhost +local:\$$USER"                              >> $(TGT_NAME)
endif
	@echo "docker run -it --rm \\"                            >> $(TGT_NAME)
	@echo "    -v $(WORK_PATH_HOST):/root/$(WORK_DIR) \\"     >> $(TGT_NAME)
ifeq ($(PLATFORM), j7)
	@echo "    -v $(REPO_PATH):$(REPO_PATH) \\"               >> $(TGT_NAME)
endif
ifeq ($(PLATFORM), j7)
	@echo "    -v $(WORK_PATH_HOST)/.ros:/root/.ros \\"       >> $(TGT_NAME)
	@echo "    -v /opt/edgeai-gst-apps:/opt/edgeai-gst-apps \\"     >> $(TGT_NAME)
	@echo "    -v /opt/imaging:/opt/imaging \\"               >> $(TGT_NAME)
	@echo "    -v /opt/model_zoo:/opt/model_zoo \\"           >> $(TGT_NAME)
endif
	@echo "    -v /dev:/dev \\"                               >> $(TGT_NAME)
	@echo "    --privileged \\"                               >> $(TGT_NAME)
	@echo "    --network host \\"                             >> $(TGT_NAME)
	@echo "    --env USE_PROXY=\$$USE_PROXY \\"               >> $(TGT_NAME)
ifeq ($(PLATFORM), pc)
ifeq ($(ROS_VER), 1)
	@echo "    --env J7_IP_ADDR=\$$J7_IP_ADDR \\"             >> $(TGT_NAME)
	@echo "    --env PC_IP_ADDR=\$$PC_IP_ADDR \\"             >> $(TGT_NAME)
endif
	@echo "    --env SDK_DIR=/root/$(WORK_DIR)/$(WS_NAME)/src/$(REPO_NAME) \\" >> $(TGT_NAME)
endif
ifeq ($(PLATFORM), j7)
	@echo "    --env TIVA_LIB_VER=$(TIVA_LIB_VER) \\"         >> $(TGT_NAME)
ifeq ($(ROS_VER), 1)
	@echo "    --env J7_IP_ADDR=\$$IP_ADDR \\"                >> $(TGT_NAME)
endif
	@echo "    --env-file \$$DOCKER_DIR/$(ENV_FILE) \\"       >> $(TGT_NAME)
	@echo "    --device-cgroup-rule='c 235:* rmw' \\"         >> $(TGT_NAME)
endif
ifeq ($(PLATFORM), pc)
ifeq ($(GPUS), y)
	@echo "    --gpus all \\"                                 >> $(TGT_NAME)
	@echo "    --env=NVIDIA_VISIBLE_DEVICES=all \\"           >> $(TGT_NAME)
	@echo "    --env=NVIDIA_DRIVER_CAPABILITIES=all \\"       >> $(TGT_NAME)
endif
	@echo "    --env='DISPLAY' \\"                            >> $(TGT_NAME)
	@echo "    --env='QT_X11_NO_MITSHM=1' \\"                 >> $(TGT_NAME)
	@echo "    --volume='/tmp/.X11-unix:/tmp/.X11-unix:rw' \\">> $(TGT_NAME)
endif
	@echo "      \$$DOCKER_TAG \$$CMD"                        >> $(TGT_NAME)
ifeq ($(PLATFORM), pc)
	@echo "xhost -local:\$$USER"                              >> $(TGT_NAME)
endif
	@chmod +x $(TGT_NAME)
	@echo "$(TGT_NAME) is generated"

scripts: scripts_$(PLATFORM)
scripts_$(PLATFORM): docker_build_$(PLATFORM) docker_run_$(PLATFORM)

docker_build_j7:
	@$(MAKE) docker_build TGT_NAME=docker_build_$(ROS_TAG).sh

docker_run_j7:
	@$(MAKE) docker_run TGT_NAME=docker_run_$(ROS_TAG).sh

docker_build_pc:
	@$(MAKE) docker_build TGT_NAME=docker_build_$(ROS_TAG).sh

docker_run_pc:
	@$(MAKE) docker_run TGT_NAME=docker_run_$(ROS_TAG).sh

## data files
data_download: rosbag_download vl_data_download calib_download

rosbag_download: check_data_dir
	@echo "[rosbag_download] Downloading ROSBAG files ..."
	@rm -rf  $(DATA_DIR)/ros_bag
	@mkdir -p $(DATA_DIR)/ros_bag
	@wget -O $(DATA_DIR)/$(ROSBAG_ZED_FILE) "$(ROSBAG_ZED_URL)"
	@echo "Extracting..."
	@tar xzf $(DATA_DIR)/$(ROSBAG_ZED_FILE) -C $(DATA_DIR)/ros_bag
	@rm -f $(DATA_DIR)/$(ROSBAG_ZED_FILE)
	@wget -O $(DATA_DIR)/$(ROSBAG_6DPOSE_FILE) "$(ROSBAG_6DPOSE_URL)"
	@echo "Extracting..."
	@tar xzf $(DATA_DIR)/$(ROSBAG_6DPOSE_FILE) -C $(DATA_DIR)/ros_bag
	@rm -f $(DATA_DIR)/$(ROSBAG_6DPOSE_FILE)
	@wget -O $(DATA_DIR)/$(ROSBAG_HUMANPOSE_FILE) "$(ROSBAG_HUMANPOSE_URL)"
	@echo "Extracting..."
	@tar xzf $(DATA_DIR)/$(ROSBAG_HUMANPOSE_FILE) -C $(DATA_DIR)/ros_bag
	@rm -f $(DATA_DIR)/$(ROSBAG_HUMANPOSE_FILE)
	@wget -O $(DATA_DIR)/$(ROSBAG_ODRADAR_FILE) "$(ROSBAG_ODRADAR_URL)"
	@echo "Extracting..."
	@tar xzf $(DATA_DIR)/$(ROSBAG_ODRADAR_FILE) -C $(DATA_DIR)/ros_bag
	@rm -f $(DATA_DIR)/$(ROSBAG_ODRADAR_FILE)
	@echo "[rosbag_download] Done"

vl_data_download: check_data_dir
ifneq ($(SOC),am62a)
	@echo "[vl_data_download] Downloading the VL data files ..."
	@wget -O $(DATA_DIR)/$(VISLOC_DATA_FILE) "$(VISLOC_DATA_URL)"
	@rm -rf  $(DATA_DIR)/visual_localization_data
	@tar xzf $(DATA_DIR)/$(VISLOC_DATA_FILE) -C $(DATA_DIR)
	@rm -f $(DATA_DIR)/$(VISLOC_DATA_FILE)
	@echo "[vl_data_download] Done"
endif

lidar2d_download: check_data_dir
	@echo "[lidar2d_download] Downloading Lidar data ..."
	@wget -O $(DATA_DIR)/$(LIDAR2D_FILE) "$(LIDAR2D_LOC_URL)"
	@rm -rf  $(DATA_DIR)/lidar_2d
	@tar xzf $(DATA_DIR)/$(LIDAR2D_FILE) -C $(DATA_DIR)
	@rm -f $(DATA_DIR)/$(LIDAR2D_FILE)
	@echo "[lidar2d_download] Done"

calib_download: check_data_dir
	@echo "[calib_download] Downloading camera calib data ..."
	@wget -O $(DATA_DIR)/$(CALIB_IMGS_FILE) "$(CALIB_IMGS_URL)"
	@rm -rf  $(DATA_DIR)/calib_imgs_example
	@tar xzf $(DATA_DIR)/$(CALIB_IMGS_FILE) -C $(DATA_DIR)
	@rm -f $(DATA_DIR)/$(CALIB_IMGS_FILE)
	@echo "[calib_download] Done"

check_data_dir:
	@if [ ! -d $(DATA_DIR) ]; then \
		mkdir -p $(DATA_DIR); \
		echo "mkdir -p $(DATA_DIR)"; \
	else \
		echo "Ok, $(DATA_DIR) exists"; \
	fi

data_clean:
	rm -rf $(DATA_DIR)

## .PHONY
.PHONY: ip_show \
		rosbag_download \
		vl_data_download \
		lidar2d_download \
		check_data_dir
