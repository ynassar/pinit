echo "Installing ros base follwoing ros kinetic online guide..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get -y -q update 

ros_setup_packages=(
	"ros-kinetic-ros-base"
	"python-rosinstall"
	"python-rosinstall-generator"
	"python-wstool"
	"build-essential"
)

sudo apt-get install -q -y ${ros_setup_packages[@]}

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

echo "Installing imu dependancies..."
imu_packages=(
	"libi2c-dev"
	"libeigen3-dev"
	"libboost-program-options-dev"
	"ros-kinetic-imu-tools"
)

sudo apt-get install -q -y ${imu_packages[@]}

echo "Installing ros messages"
sudo apt install -q -y ros-kinetic-common-msgs
