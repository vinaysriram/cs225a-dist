set -e

# Update sai2-common
cd sai2-common
git pull origin master
mkdir -p build_rel
cd build_rel
cmake ..
make -j4
cd ../..

# Build cs225a
mkdir -p build
cd build
cmake ..
make -j4
cd ..

# Download resource files
mkdir -p resources
cd resources
if [ ! -d "puma_graphics" ]; then
    curl -L http://cs.stanford.edu/groups/manips/teaching/cs225a/resources/puma_graphics.zip -o puma_graphics.zip
    unzip puma_graphics.zip
    rm puma_graphics.zip
fi
if [ ! -d "kuka_iiwa_graphics" ]; then
    curl -L http://cs.stanford.edu/groups/manips/teaching/cs225a/resources/kuka_iiwa_graphics.zip -o kuka_iiwa_graphics.zip
    unzip kuka_iiwa_graphics.zip
    rm kuka_iiwa_graphics.zip
fi
if [ ! -d "sawyer_graphics" ]; then
    curl -L http://cs.stanford.edu/groups/manips/teaching/cs225a/resources/sawyer_graphics.zip -o sawyer_graphics.zip
    unzip sawyer_graphics.zip
    rm sawyer_graphics.zip
fi
cd ..

cd bin
if [ -f "hw1" ]; then
    cd resources/hw1
    if [ ! -e "puma_graphics" ]; then
	ln -s ../../../resources/puma_graphics .
    fi
    cd ../..
fi
if [ -f "hw2" ]; then
	cd resources/hw2
	if [ ! -e "kuka_iiwa_graphics" ]; then
		ln -s ../../../resources/kuka_iiwa_graphics .
	fi
	cd ../..
fi
if [ -f "hw3" ]; then
    cd resources/hw3
    if [ ! -e "kuka_iiwa_graphics" ]; then
	ln -s ../../../resources/kuka_iiwa_graphics .
    fi
    cd ../..
fi
if [ -f "demo_project" ]; then
    cd resources/demo_project
    if [ ! -e "kuka_iiwa_graphics" ]; then
	ln -s ../../../resources/kuka_iiwa_graphics .
    fi
    cd ../..
fi
if [ -f "project_controller" ]; then
    cd resources/project_controller
    if [ ! -e "sawyer_graphics" ]; then
	ln -s ../../../resources/sawyer_graphics .
    fi
    cd ../..
fi
cd ..

# Insert helper scripts into bin directory
cd bin

# Make script
cat <<EOF > make.sh
cd ..
mkdir -p build
cd build
cmake ..
make -j4
cd ../bin
EOF
chmod +x make.sh

# Project Controller Script
if [ -f 'project_controller' ]; then
    cat <<EOF > init_controller.sh
redis-cli flushall
redis-cli set cs225a::robot::sawyer::tasks::kp_pos 300
redis-cli set cs225a::robot::sawyer::tasks::kv_pos 40
redis-cli set cs225a::robot::sawyer::tasks::kp_ori 300
redis-cli set cs225a::robot::sawyer::tasks::kv_ori 30
redis-cli set cs225a::robot::sawyer::tasks::kp_joint 0
redis-cli set cs225a::robot::sawyer::tasks::kv_joint 30
redis-cli set cs225a::robot::sawyer::tasks::kp_joint_init 30
redis-cli set cs225a::robot::sawyer::tasks::kv_joint_init 10
redis-cli set cs225a::robot::sawyer::tasks::jt_pos_des "-0.001125 -1.1782490234375 -0.002130859375 2.17892578125 0.00065234375 0.568098632812 3.313166015625"
./run_controller.sh project_controller resources/project_controller/world.urdf resources/project_controller/sawyer.urdf sawyer
EOF
fi

# Run hw0 script
if [ -f "hw0" ]; then
	cat <<EOF > run_hw0.sh
# This script calls ./visualizer and ./hw0 simultaneously.
# All arguments to this script will be passed onto ./hw0

trap 'kill %1' SIGINT
./visualizer resources/hw0/world.urdf resources/hw0/RRPbot.urdf RRPbot & ./hw0 "\$@"
EOF
	chmod +x run_hw0.sh
fi

# Run generic controller script
if [ -f "hw1" ]; then
	cat <<EOF > run_controller.sh
if [ "\$#" -lt 4 ]; then
	cat <<EOM
This script calls ./visualizer, ./simulator, and the specified controller simultaneously.
All the arguments after the controller will be passed directly to it.

Usage: sh run.sh <controller-executable> <path-to-world.urdf> <path-to-robot.urdf> <robot-name> <extra_controller_args>
EOM
else
	trap 'kill %1; kill %2' SIGINT
	trap 'kill %1; kill %2' EXIT
	./simulator \$2 \$3 \$4 > simulator.log & ./visualizer_project \$2 \$3 \$4 > visualizer.log & ./"\$@"
	# ./visualizer \$2 \$3 \$4 & ./simulator \$2 \$3 \$4 & ./"\$@"
fi
EOF
	chmod +x run_controller.sh
fi

cd ..
