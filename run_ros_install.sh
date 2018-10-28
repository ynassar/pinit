echo "Installing ros base following ros kinetic online guide..."
echo "Updating ubuntu source list.."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get -y -qq update 

ros_setup_packages=(
	"ros-kinetic-ros-base"
	"python-rosinstall"
	"python-rosinstall-generator"
	"python-wstool"
	"build-essential"
)

echo "Installing ros base packages..."
sudo apt-get install -qq -y ${ros_setup_packages[@]}


echo "Other ros initilizations..."
source /opt/ros/kinetic/setup.bash
sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

echo "Installing ros messages..."
sudo apt install -qq -y ros-kinetic-common-msgs

echo "Installing ros dependancy packages"
rosdep install --from-paths ros_ws/src -i -y

echo "Building ros package.."
cd ros_ws
catkin_make


echo "Done!"

